#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = moveit_visual_tools;

enum ShapeType
{
    BOX = 0,
    SPHERE = 1,
    CYLINDER = 2,
};

struct Obstacle
{
    ShapeType type;
    double x;
    double y;
    double z;
    std::vector<double> dims;

    Obstacle() : type(BOX), x(0.0), y(0.0), z(0.0), dims(3, 0.0) {}
};

struct SceneConfig
{
    std::string scene_type = "mixed";
    int random_obstacle_count = 15;
    double shelf_open_side_sign = -1.0;

    double shelf_center_x = 0.55;
    double shelf_center_y = 0.0;
    double shelf_center_z = 0.45;
    double shelf_width = 0.80;
    double shelf_depth = 0.30;
    double shelf_height = 0.80;
    double shelf_board_thickness = 0.035;
    bool shelf_add_table = true;
    bool shelf_add_back_board = true;
    int shelf_box_count = 3;

    double table_center_x = 0.50;
    double table_center_y = 0.0;
    double table_top_z = 0.26;
    double table_size_x = 0.70;
    double table_size_y = 0.75;
    double table_thickness = 0.04;
    double table_box_height = 0.12;
    int table_box_count = 4;
    bool table_add_back_guard = true;
    bool table_add_side_guards = false;
};

planning_scene::PlanningScenePtr g_planning_scene;
robot_state::RobotStatePtr g_current_robot_state;
const robot_state::JointModelGroup *g_joint_model_group = nullptr;

Obstacle makeBoxObstacle(double x, double y, double z, double sx, double sy, double sz)
{
    Obstacle obs;
    obs.type = BOX;
    obs.x = x;
    obs.y = y;
    obs.z = z;
    obs.dims = {sx, sy, sz};
    return obs;
}

Obstacle makeCylinderObstacle(double x, double y, double z, double height, double radius)
{
    Obstacle obs;
    obs.type = CYLINDER;
    obs.x = x;
    obs.y = y;
    obs.z = z;
    obs.dims = {height, radius, 0.0};
    return obs;
}

int findNextSceneId(const std::string &csv_path)
{
    std::ifstream in(csv_path);
    if (!in.is_open())
    {
        return 0;
    }

    int max_scene_id = -1;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::stringstream ss(line);
        std::string first_col;
        if (!std::getline(ss, first_col, ','))
        {
            continue;
        }

        try
        {
            max_scene_id = std::max(max_scene_id, std::stoi(first_col));
        }
        catch (const std::exception &)
        {
            continue;
        }
    }

    return max_scene_id + 1;
}

void addObstacleToScene(const std::string &frame, int obstacle_id, const Obstacle &obstacle_info)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame;
    collision_object.id = "obstacle_" + std::to_string(obstacle_id);

    shape_msgs::SolidPrimitive primitive;
    if (obstacle_info.type == BOX)
    {
        primitive.type = shape_msgs::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = obstacle_info.dims[0];
        primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = obstacle_info.dims[1];
        primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = obstacle_info.dims[2];
    }
    else if (obstacle_info.type == SPHERE)
    {
        primitive.type = shape_msgs::SolidPrimitive::SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = obstacle_info.dims[0];
    }
    else
    {
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = obstacle_info.dims[0];
        primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = obstacle_info.dims[1];
    }

    geometry_msgs::Pose pose;
    pose.position.x = obstacle_info.x;
    pose.position.y = obstacle_info.y;
    pose.position.z = obstacle_info.z;
    pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    g_planning_scene->processCollisionObjectMsg(collision_object);
}

void writeObstacleCsvRow(std::ofstream &env_file, int scene_id, const Obstacle &obs)
{
    env_file << scene_id << "," << obs.type << ","
             << obs.x << "," << obs.y << "," << obs.z << ","
             << obs.dims[0] << "," << obs.dims[1] << "," << obs.dims[2] << "\n";
}

void addObstacleCollectionToScene(const std::vector<Obstacle> &obstacles,
                                  std::ofstream &env_file,
                                  int scene_id,
                                  const std::string &frame = "base_link")
{
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        addObstacleToScene(frame, static_cast<int>(i), obstacles[i]);
        writeObstacleCsvRow(env_file, scene_id, obstacles[i]);
    }
}

std::vector<Obstacle> makeShelfApplicationScene(double cx,
                                                double cy,
                                                double cz,
                                                double width,
                                                double depth,
                                                double height,
                                                double board_thickness,
                                                double open_side_sign,
                                                bool add_table,
                                                bool add_back_board,
                                                int box_count)
{
    std::vector<Obstacle> obstacles;

    const double shelf_side_sign = open_side_sign >= 0.0 ? 1.0 : -1.0;
    const double back_side_sign = -shelf_side_sign;

    if (add_table)
    {
        obstacles.push_back(makeCylinderObstacle(0.0, 0.0, -0.01, 0.02, 1.0));
    }

    const double half_width = width * 0.5;
    const double half_depth = depth * 0.5;
    const double half_height = height * 0.5;

    obstacles.push_back(makeBoxObstacle(cx, cy - half_width, cz, depth, board_thickness, height));
    obstacles.push_back(makeBoxObstacle(cx, cy + half_width, cz, depth, board_thickness, height));
    obstacles.push_back(makeBoxObstacle(cx, cy, cz - half_height, depth, width, board_thickness));
    obstacles.push_back(makeBoxObstacle(cx, cy, cz + half_height, depth, width, board_thickness));
    obstacles.push_back(makeBoxObstacle(cx, cy, cz, depth, width, board_thickness));

    if (add_back_board)
    {
        obstacles.push_back(makeBoxObstacle(cx + back_side_sign * half_depth, cy, cz, board_thickness, width, height));
    }

    const double box_sx = std::min(0.16, depth * 0.55);
    const double box_sy = std::min(0.18, width * 0.22);
    const double box_sz = std::min(0.12, height * 0.18);
    const double clearance = 0.01;

    const std::vector<double> level_z = {
        cz - half_height + board_thickness + box_sz * 0.5 + clearance,
        cz + board_thickness + box_sz * 0.5 + clearance,
    };
    const std::vector<double> slot_y = {
        cy - width * 0.25,
        cy,
        cy + width * 0.25,
    };

    const int clamped_box_count = std::max(0, std::min(box_count, 6));
    for (int i = 0; i < clamped_box_count; ++i)
    {
        const double z = level_z[static_cast<size_t>(i / 3)];
        const double y = slot_y[static_cast<size_t>(i % 3)];
        obstacles.push_back(makeBoxObstacle(cx + shelf_side_sign * depth * 0.15, y, z, box_sx, box_sy, box_sz));
    }

    return obstacles;
}

std::vector<Obstacle> makeTableBoxApplicationScene(double cx,
                                                   double cy,
                                                   double table_top_z,
                                                   double table_size_x,
                                                   double table_size_y,
                                                   double table_thickness,
                                                   double box_height,
                                                   int box_count,
                                                   bool add_back_guard,
                                                   bool add_side_guards)
{
    std::vector<Obstacle> obstacles;

    const double clamped_table_size_x = std::max(0.20, table_size_x);
    const double clamped_table_size_y = std::max(0.20, table_size_y);
    const double clamped_table_thickness = std::max(0.01, table_thickness);
    const double clamped_box_height = std::max(0.04, box_height);

    obstacles.push_back(makeBoxObstacle(cx, cy, table_top_z - clamped_table_thickness * 0.5,
                                        clamped_table_size_x, clamped_table_size_y,
                                        clamped_table_thickness));

    const double guard_thickness = 0.035;
    const double guard_height = 0.18;
    const double guard_z = table_top_z + guard_height * 0.5;
    if (add_back_guard)
    {
        obstacles.push_back(makeBoxObstacle(cx - clamped_table_size_x * 0.5 + guard_thickness * 0.5,
                                            cy, guard_z, guard_thickness,
                                            clamped_table_size_y, guard_height));
    }
    if (add_side_guards)
    {
        obstacles.push_back(makeBoxObstacle(cx, cy - clamped_table_size_y * 0.5 + guard_thickness * 0.5,
                                            guard_z, clamped_table_size_x,
                                            guard_thickness, guard_height));
        obstacles.push_back(makeBoxObstacle(cx, cy + clamped_table_size_y * 0.5 - guard_thickness * 0.5,
                                            guard_z, clamped_table_size_x,
                                            guard_thickness, guard_height));
    }

    const double clearance = 0.01;
    const double object_z = table_top_z + clamped_box_height * 0.5 + clearance;
    const double box_sx = std::min(0.16, clamped_table_size_x * 0.24);
    const double box_sy = std::min(0.16, clamped_table_size_y * 0.24);
    const std::vector<std::pair<double, double>> slots = {
        {cx + clamped_table_size_x * 0.18, cy - clamped_table_size_y * 0.22},
        {cx + clamped_table_size_x * 0.20, cy + clamped_table_size_y * 0.18},
        {cx - clamped_table_size_x * 0.05, cy},
        {cx + clamped_table_size_x * 0.32, cy},
        {cx - clamped_table_size_x * 0.18, cy + clamped_table_size_y * 0.22},
        {cx - clamped_table_size_x * 0.18, cy - clamped_table_size_y * 0.18},
    };

    const int clamped_box_count = std::max(0, std::min(box_count, static_cast<int>(slots.size())));
    for (int i = 0; i < clamped_box_count; ++i)
    {
        obstacles.push_back(makeBoxObstacle(slots[static_cast<size_t>(i)].first,
                                            slots[static_cast<size_t>(i)].second,
                                            object_z, box_sx, box_sy,
                                            clamped_box_height));
    }

    return obstacles;
}

std::vector<Obstacle> makeRandomScene(double max_reach, int num_obstacles)
{
    std::vector<Obstacle> obstacles;
    obstacles.push_back(makeCylinderObstacle(0.0, 0.0, -0.0101, 0.02, 1.0));

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> radius_dist(0.20, std::max(0.35, max_reach * 0.95));
    std::uniform_real_distribution<> angle_dist(-M_PI, M_PI);
    std::uniform_real_distribution<> z_dist(0.0, 0.6);
    std::uniform_real_distribution<> size_dist(0.05, 0.30);
    std::uniform_real_distribution<> radius_size_dist(0.05, 0.15);
    std::uniform_int_distribution<> type_dist(0, 2);

    for (int i = 0; i < num_obstacles; ++i)
    {
        Obstacle obs;
        const double r = radius_dist(gen);
        const double theta = angle_dist(gen);
        obs.x = r * std::cos(theta);
        obs.y = r * std::sin(theta);
        obs.z = z_dist(gen);
        obs.type = static_cast<ShapeType>(type_dist(gen));

        if (obs.type == BOX)
        {
            obs.dims[0] = size_dist(gen);
            obs.dims[1] = size_dist(gen);
            obs.dims[2] = size_dist(gen);
        }
        else if (obs.type == SPHERE)
        {
            obs.dims[0] = radius_size_dist(gen);
        }
        else
        {
            obs.dims[0] = size_dist(gen);
            obs.dims[1] = radius_size_dist(gen);
        }

        obstacles.push_back(obs);
    }

    return obstacles;
}

std::vector<Obstacle> buildSceneObstacles(const SceneConfig &cfg, int scene_id, double max_reach)
{
    if (cfg.scene_type == "shelf")
    {
        return makeShelfApplicationScene(cfg.shelf_center_x,
                                         cfg.shelf_center_y,
                                         cfg.shelf_center_z,
                                         cfg.shelf_width,
                                         cfg.shelf_depth,
                                         cfg.shelf_height,
                                         cfg.shelf_board_thickness,
                                         cfg.shelf_open_side_sign,
                                         cfg.shelf_add_table,
                                         cfg.shelf_add_back_board,
                                         cfg.shelf_box_count);
    }

    if (cfg.scene_type == "table_box" || cfg.scene_type == "table_place")
    {
        return makeTableBoxApplicationScene(cfg.table_center_x,
                                            cfg.table_center_y,
                                            cfg.table_top_z,
                                            cfg.table_size_x,
                                            cfg.table_size_y,
                                            cfg.table_thickness,
                                            cfg.table_box_height,
                                            cfg.table_box_count,
                                            cfg.table_add_back_guard,
                                            cfg.table_add_side_guards);
    }

    if (cfg.scene_type == "mixed")
    {
        if (scene_id % 2 == 0)
        {
            return makeShelfApplicationScene(cfg.shelf_center_x,
                                             cfg.shelf_center_y,
                                             cfg.shelf_center_z,
                                             cfg.shelf_width,
                                             cfg.shelf_depth,
                                             cfg.shelf_height,
                                             cfg.shelf_board_thickness,
                                             cfg.shelf_open_side_sign,
                                             cfg.shelf_add_table,
                                             cfg.shelf_add_back_board,
                                             cfg.shelf_box_count);
        }

        return makeTableBoxApplicationScene(cfg.table_center_x,
                                            cfg.table_center_y,
                                            cfg.table_top_z,
                                            cfg.table_size_x,
                                            cfg.table_size_y,
                                            cfg.table_thickness,
                                            cfg.table_box_height,
                                            cfg.table_box_count,
                                            cfg.table_add_back_guard,
                                            cfg.table_add_side_guards);
    }

    return makeRandomScene(max_reach, cfg.random_obstacle_count);
}

double computeMaxReach(const robot_model::JointModelGroup *group)
{
    double max_reach = 0.0;
    const std::vector<const robot_model::LinkModel *> &links = group->getLinkModels();
    for (const auto *link : links)
    {
        max_reach += link->getJointOriginTransform().translation().norm();
    }
    return max_reach;
}

void getRobotConf(ob::RealVectorBounds &bounds, int dim)
{
    const std::vector<std::string> &vars = g_joint_model_group->getVariableNames();
    const robot_model::RobotModel &robot_model = g_joint_model_group->getParentModel();

    for (unsigned int i = 0; i < dim; ++i)
    {
        const robot_model::VariableBounds &b = robot_model.getVariableBounds(vars[i]);
        if (b.position_bounded_)
        {
            bounds.setLow(i, b.min_position_);
            bounds.setHigh(i, b.max_position_);
        }
        else
        {
            bounds.setLow(i, -M_PI);
            bounds.setHigh(i, M_PI);
        }
    }
}

bool isStateValid(const ob::State *state)
{
    const auto *joint_state = state->as<ob::RealVectorStateSpace::StateType>();
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, joint_state->values);

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.contacts = false;
    g_planning_scene->checkCollision(c_req, c_res, *g_current_robot_state);

    return !c_res.collision;
}

Eigen::Vector3d getEndEffectorPosition(const ob::State *state)
{
    const auto *joint_state = state->as<ob::RealVectorStateSpace::StateType>();
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, joint_state->values);
    g_current_robot_state->updateLinkTransforms();

    const std::string &ee_link_name = g_joint_model_group->getLinkModelNames().back();
    return g_current_robot_state->getGlobalLinkTransform(ee_link_name).translation();
}

struct EeRegion
{
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
};

bool sampleStateInEeRegion(const ob::StateSpacePtr &space,
                           const EeRegion &region,
                           ob::ScopedState<> &sample,
                           int max_attempts = 5000)
{
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        sample.random();
        if (!isStateValid(sample.get()))
        {
            continue;
        }

        const Eigen::Vector3d ee_pos = getEndEffectorPosition(sample.get());
        if (ee_pos.x() >= region.min_x && ee_pos.x() <= region.max_x &&
            ee_pos.y() >= region.min_y && ee_pos.y() <= region.max_y &&
            ee_pos.z() >= region.min_z && ee_pos.z() <= region.max_z)
        {
            return true;
        }
    }

    return false;
}

bool sampleTaskEndpoints(const SceneConfig &cfg,
                         const ob::SpaceInformationPtr &si,
                         const ob::StateSpacePtr &space,
                         ob::ScopedState<> &start,
                         ob::ScopedState<> &goal)
{
    if (cfg.scene_type == "shelf" || cfg.scene_type == "mixed")
    {
        const double open_sign = cfg.shelf_open_side_sign >= 0.0 ? 1.0 : -1.0;
        const double front_x = cfg.shelf_center_x + open_sign * (cfg.shelf_depth * 0.5 + 0.18);
        const double inside_x = cfg.shelf_center_x + open_sign * (cfg.shelf_depth * 0.18);
        const double side_span = cfg.shelf_width * 0.34;
        const double z_base = cfg.shelf_center_z - cfg.shelf_height * 0.5;

        const EeRegion start_region{
            front_x - 0.10, front_x + 0.10,
            cfg.shelf_center_y - side_span, cfg.shelf_center_y + side_span,
            z_base + 0.10, z_base + cfg.shelf_height * 0.92};
        const EeRegion goal_region{
            inside_x - 0.10, inside_x + 0.10,
            cfg.shelf_center_y - side_span * 0.75, cfg.shelf_center_y + side_span * 0.75,
            z_base + cfg.shelf_height * 0.12, z_base + cfg.shelf_height * 0.80};

        if (sampleStateInEeRegion(space, start_region, start) &&
            sampleStateInEeRegion(space, goal_region, goal))
        {
            return true;
        }
        return false;
    }

    if (cfg.scene_type == "table_box" || cfg.scene_type == "table_place")
    {
        const double table_top = cfg.table_top_z;
        const double half_x = cfg.table_size_x * 0.5;
        const double half_y = cfg.table_size_y * 0.5;

        const EeRegion start_region{
            cfg.table_center_x - half_x - 0.12, cfg.table_center_x + half_x + 0.06,
            cfg.table_center_y - half_y, cfg.table_center_y + half_y,
            table_top + 0.08, table_top + 0.45};
        const EeRegion goal_region{
            cfg.table_center_x - half_x * 0.10, cfg.table_center_x + half_x * 0.35,
            cfg.table_center_y - half_y * 0.40, cfg.table_center_y + half_y * 0.40,
            table_top + cfg.table_box_height * 0.35, table_top + cfg.table_box_height * 0.95};

        if (sampleStateInEeRegion(space, start_region, start) &&
            sampleStateInEeRegion(space, goal_region, goal))
        {
            return true;
        }
        return false;
    }

    return false;
}

void customSmoothPath(og::PathGeometric &path, const ob::SpaceInformationPtr &si, int max_iterations = 3)
{
    if (path.getStateCount() < 3)
    {
        return;
    }

    ob::StateSpacePtr space = si->getStateSpace();
    ob::State *temp1 = space->allocState();
    ob::State *temp2 = space->allocState();

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        bool changed = false;
        og::PathGeometric new_path(si);
        new_path.append(path.getState(0));

        for (size_t i = 1; i + 1 < path.getStateCount(); ++i)
        {
            const ob::State *prev = path.getState(i - 1);
            const ob::State *curr = path.getState(i);
            const ob::State *next = path.getState(i + 1);

            space->interpolate(prev, curr, 0.7, temp1);
            space->interpolate(curr, next, 0.3, temp2);
            if (si->checkMotion(temp1, temp2))
            {
                new_path.append(temp1);
                new_path.append(temp2);
                changed = true;
            }
            else
            {
                space->interpolate(prev, curr, 0.9, temp1);
                space->interpolate(curr, next, 0.1, temp2);
                if (si->checkMotion(temp1, temp2))
                {
                    new_path.append(temp1);
                    new_path.append(temp2);
                    changed = true;
                }
                else
                {
                    new_path.append(curr);
                }
            }
        }

        new_path.append(path.getState(path.getStateCount() - 1));
        path = new_path;
        if (!changed)
        {
            break;
        }
    }

    space->freeState(temp1);
    space->freeState(temp2);
}

bool isPathSafe(const og::PathGeometric &path, const ob::SpaceInformationPtr &si)
{
    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        if (!si->isValid(path.getState(i)))
        {
            return false;
        }
    }

    for (size_t i = 0; i + 1 < path.getStateCount(); ++i)
    {
        if (!si->checkMotion(path.getState(i), path.getState(i + 1)))
        {
            return false;
        }
    }

    return true;
}

void visualizePath(og::PathGeometric &path,
                   rvt::MoveItVisualToolsPtr &visual_tools,
                   const robot_state::JointModelGroup *group)
{
    if (!visual_tools)
    {
        return;
    }

    const std::vector<std::string> &link_names = group->getLinkModelNames();
    if (link_names.empty())
    {
        return;
    }

    const std::string &ee_link_name = link_names.back();
    std::vector<geometry_msgs::Point> path_points;

    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        g_current_robot_state->setJointGroupPositions(group, state->values);
        g_current_robot_state->updateLinkTransforms();
        const Eigen::Isometry3d &pose = g_current_robot_state->getGlobalLinkTransform(ee_link_name);

        geometry_msgs::Point p;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        path_points.push_back(p);
    }

    visual_tools->publishPath(path_points, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    visual_tools->trigger();
}

bool sampleValidEndpoints(const ob::SpaceInformationPtr &si,
                          const ob::StateSpacePtr &space,
                          ob::ScopedState<> &start,
                          ob::ScopedState<> &goal)
{
    bool valid_endpoints = false;
    bool is_hard_sample = false;
    int attempts = 0;

    while (!valid_endpoints && ros::ok() && attempts < 1000)
    {
        attempts++;
        start.random();
        goal.random();

        if (isStateValid(start.get()) && isStateValid(goal.get()))
        {
            if (!si->checkMotion(start.get(), goal.get()))
            {
                is_hard_sample = true;
            }

            if (!is_hard_sample && (rand() % 100) < 80)
            {
                continue;
            }

            valid_endpoints = true;
        }
    }

    return valid_endpoints;
}

bool sampleEndpointsForScene(const SceneConfig &cfg,
                             const ob::SpaceInformationPtr &si,
                             const ob::StateSpacePtr &space,
                             ob::ScopedState<> &start,
                             ob::ScopedState<> &goal)
{
    if (cfg.scene_type == "shelf" || cfg.scene_type == "table_box" || cfg.scene_type == "table_place" || cfg.scene_type == "mixed")
    {
        if (sampleTaskEndpoints(cfg, si, space, start, goal))
        {
            return true;
        }
    }

    return sampleValidEndpoints(si, space, start, goal);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ompl_application_data_generator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    bool debug_mode = false;
    private_nh.param("debug", debug_mode, false);

    SceneConfig cfg;
    private_nh.param("scene_type", cfg.scene_type, cfg.scene_type);
    private_nh.param("random_obstacle_count", cfg.random_obstacle_count, cfg.random_obstacle_count);
    private_nh.param("shelf_open_side_sign", cfg.shelf_open_side_sign, cfg.shelf_open_side_sign);
    private_nh.param("shelf_center_x", cfg.shelf_center_x, cfg.shelf_center_x);
    private_nh.param("shelf_center_y", cfg.shelf_center_y, cfg.shelf_center_y);
    private_nh.param("shelf_center_z", cfg.shelf_center_z, cfg.shelf_center_z);
    private_nh.param("shelf_width", cfg.shelf_width, cfg.shelf_width);
    private_nh.param("shelf_depth", cfg.shelf_depth, cfg.shelf_depth);
    private_nh.param("shelf_height", cfg.shelf_height, cfg.shelf_height);
    private_nh.param("shelf_board_thickness", cfg.shelf_board_thickness, cfg.shelf_board_thickness);
    private_nh.param("shelf_add_table", cfg.shelf_add_table, cfg.shelf_add_table);
    private_nh.param("shelf_add_back_board", cfg.shelf_add_back_board, cfg.shelf_add_back_board);
    private_nh.param("shelf_box_count", cfg.shelf_box_count, cfg.shelf_box_count);
    private_nh.param("table_center_x", cfg.table_center_x, cfg.table_center_x);
    private_nh.param("table_center_y", cfg.table_center_y, cfg.table_center_y);
    private_nh.param("table_top_z", cfg.table_top_z, cfg.table_top_z);
    private_nh.param("table_size_x", cfg.table_size_x, cfg.table_size_x);
    private_nh.param("table_size_y", cfg.table_size_y, cfg.table_size_y);
    private_nh.param("table_thickness", cfg.table_thickness, cfg.table_thickness);
    private_nh.param("table_box_height", cfg.table_box_height, cfg.table_box_height);
    private_nh.param("table_box_count", cfg.table_box_count, cfg.table_box_count);
    private_nh.param("table_add_back_guard", cfg.table_add_back_guard, cfg.table_add_back_guard);
    private_nh.param("table_add_side_guards", cfg.table_add_side_guards, cfg.table_add_side_guards);

    int total_scenes = 100;
    int paths_per_scene = 20;
    double planning_timeout = 25.0;
    bool overwrite_output = false;
    std::string output_dir = std::string(std::getenv("HOME")) + "/aubo/arm_ws/src/motion_planning/data/";

    private_nh.param("total_scenes", total_scenes, total_scenes);
    private_nh.param("paths_per_scene", paths_per_scene, paths_per_scene);
    private_nh.param("planning_timeout", planning_timeout, planning_timeout);
    private_nh.param("output_dir", output_dir, output_dir);
    private_nh.param("overwrite_output", overwrite_output, overwrite_output);
    planning_timeout = std::max(0.1, planning_timeout);

    ROS_INFO("Debug Mode: %s", debug_mode ? "ENABLED" : "DISABLED");
    ROS_INFO("Scene type: %s", cfg.scene_type.c_str());
    ROS_INFO("Planning timeout: %.2f s/path", planning_timeout);
    ROS_INFO("Shelf open side sign: %.1f", cfg.shelf_open_side_sign);
    ROS_INFO("Output dir: %s", output_dir.c_str());

    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader loader("robot_description"); // 加载模型（from urdf）
    auto k_model = loader.getModel();
    if (!k_model)
    {
        ROS_ERROR("Failed to load robot model.");
        return -1;
    }

    g_planning_scene.reset(new planning_scene::PlanningScene(k_model));
    g_current_robot_state.reset(new robot_state::RobotState(k_model));

    std::string group_name = "arm";
    g_joint_model_group = g_current_robot_state->getJointModelGroup(group_name);
    if (!g_joint_model_group)
    {
        ROS_ERROR("Joint group '%s' not found!", group_name.c_str());
        return -1;
    }

    rvt::MoveItVisualToolsPtr visual_tools;
    ros::Publisher scene_pub;
    ros::Publisher start_state_pub;
    ros::Publisher goal_state_pub;

    if (debug_mode)
    {
        visual_tools.reset(new rvt::MoveItVisualTools(k_model->getModelFrame(), "/rviz_visual_tools", k_model));
        visual_tools->loadRemoteControl();
        visual_tools->deleteAllMarkers();
        scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
        start_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_start_state", 10, true);
        goal_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_goal_state", 10, true);
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>(g_joint_model_group->getVariableCount());
    ob::RealVectorBounds bounds(g_joint_model_group->getVariableCount());
    getRobotConf(bounds, g_joint_model_group->getVariableCount());
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(isStateValid);
    si->setup();

    double max_r = computeMaxReach(g_joint_model_group); // 计算机械臂的最大可达范围
    if (max_r < 0.5)
    {
        max_r = 0.85;
    }
    ROS_INFO("Max Reach Calculated: %.2f m", max_r);

    std::filesystem::create_directories(output_dir);
    if (!output_dir.empty() && output_dir.back() != '/')
    {
        output_dir += "/";
    }

    const std::string env_path = output_dir + "training_envs.csv";
    const std::string path_path = output_dir + "training_paths.csv";
    if (overwrite_output)
    {
        std::ofstream(env_path, std::ios::trunc).close();
        std::ofstream(path_path, std::ios::trunc).close();
        ROS_WARN("overwrite_output=true, truncated existing dataset files in %s", output_dir.c_str());
    }

    int global_scene_id = std::max(findNextSceneId(env_path), findNextSceneId(path_path));
    ROS_INFO("Starting scene_id from %d", global_scene_id);

    std::ofstream env_file(env_path, std::ios::app);
    std::ofstream path_file(path_path, std::ios::app);
    if (!env_file.is_open() || !path_file.is_open())
    {
        ROS_ERROR("Failed to open data files in %s", output_dir.c_str());
        return -1;
    }

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    for (int base_idx = 0; base_idx < total_scenes; ++base_idx)
    {
        if (!ros::ok())
        {
            break;
        }

        const int scene_id = global_scene_id + base_idx;
        ROS_INFO("Generating Scene %d / %d", base_idx + 1, total_scenes);

        g_planning_scene->removeAllCollisionObjects();
        const std::vector<Obstacle> scene_obstacles = buildSceneObstacles(cfg, scene_id, max_r);
        addObstacleCollectionToScene(scene_obstacles, env_file, scene_id); // 添加障碍物到场景和写入csv文件

        if (debug_mode)
        {
            moveit_msgs::PlanningScene scene_msg;
            g_planning_scene->getPlanningSceneMsg(scene_msg);
            scene_msg.is_diff = true;
            scene_pub.publish(scene_msg); // 发布场景到rviz
            ros::spinOnce();
            ros::Duration(0.4).sleep();

            visual_tools->deleteAllMarkers();
            visual_tools->publishText(text_pose, "Scene " + std::to_string(scene_id), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            visual_tools->trigger();
            ros::Duration(0.1).sleep();
        }

        for (int path_id = 0; path_id < paths_per_scene; ++path_id)
        {
            if (!ros::ok())
            {
                break;
            }

            if (debug_mode)
            {
                visual_tools->deleteAllMarkers();
                visual_tools->publishText(text_pose, "Scene " + std::to_string(scene_id), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
                visual_tools->trigger();
            }

            og::SimpleSetup setup(si);
            auto planner = std::make_shared<og::BITstar>(si); // 使用BIT*算法进行路径规划
            planner->setPruning(true);
            planner->setSamplesPerBatch(200);
            setup.setPlanner(planner);

            ob::ScopedState<> start(space);
            ob::ScopedState<> goal(space);
            if (!sampleEndpointsForScene(cfg, si, space, start, goal))
            {
                ROS_WARN("Failed to sample valid endpoints for scene %d path %d", scene_id, path_id);
                continue;
            }

            auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si); // 设置优化目标为路径长度
            const double dist = space->distance(start.get(), goal.get());
            objective->setCostThreshold(ob::Cost(dist * 1.15));
            setup.setOptimizationObjective(objective);
            setup.setStartAndGoalStates(start, goal);

            if (debug_mode)
            {
                // 可视化起点和终点状态
                const double *start_val = start->as<ob::RealVectorStateSpace::StateType>()->values;
                g_current_robot_state->setJointGroupPositions(g_joint_model_group, start_val);
                g_current_robot_state->updateLinkTransforms();
                moveit_msgs::DisplayRobotState start_msg;
                moveit::core::robotStateToRobotStateMsg(*g_current_robot_state, start_msg.state);
                start_state_pub.publish(start_msg);

                const std::string &ee_link = g_joint_model_group->getLinkModelNames().back();
                const Eigen::Isometry3d start_pose = g_current_robot_state->getGlobalLinkTransform(ee_link);

                const double *goal_val = goal->as<ob::RealVectorStateSpace::StateType>()->values;
                g_current_robot_state->setJointGroupPositions(g_joint_model_group, goal_val);
                g_current_robot_state->updateLinkTransforms();
                moveit_msgs::DisplayRobotState goal_msg;
                moveit::core::robotStateToRobotStateMsg(*g_current_robot_state, goal_msg.state);
                goal_state_pub.publish(goal_msg);
                const Eigen::Isometry3d goal_pose = g_current_robot_state->getGlobalLinkTransform(ee_link);

                visual_tools->publishAxisLabeled(start_pose, "Start");
                visual_tools->publishAxisLabeled(goal_pose, "Goal");
                visual_tools->trigger();
            }

            ROS_INFO("  Planning path %d ...", path_id);
            ob::PlannerStatus solved = setup.solve(planning_timeout);

            if (solved && setup.haveExactSolutionPath())
            {
                og::PathGeometric &path = setup.getSolutionPath();
                og::PathSimplifierPtr ps = setup.getPathSimplifier();
                ps->simplifyMax(path);
                customSmoothPath(path, si, 5);
                ps->smoothBSpline(path);

                const double path_len = path.length();
                unsigned int num_points = static_cast<unsigned int>(path_len / 0.05);
                num_points = std::max(num_points, static_cast<unsigned int>(100));
                path.interpolate(num_points);

                if (!isPathSafe(path, si))
                {
                    ROS_WARN("Path %d failed final safety check. Re-generating...", path_id);
                    --path_id;
                    continue;
                }

                if (debug_mode)
                {
                    visualizePath(path, visual_tools, g_joint_model_group);
                    visual_tools->prompt("Press 'Next' in RViz to continue");
                }

                const unsigned int joint_count = g_joint_model_group->getVariableCount();
                for (size_t i = 0; i < path.getStateCount(); ++i)
                {
                    const auto *joint_state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                    path_file << scene_id << "," << path_id;
                    for (unsigned int j = 0; j < joint_count; ++j)
                    {
                        path_file << "," << joint_state->values[j];
                    }
                    path_file << "\n";
                }

                ROS_INFO("  Saved %ld dense points.", path.getStateCount());
            }
            else
            {
                ROS_WARN("  Planning failed.");
            }

            setup.clear();
        }
    }

    env_file.close();
    path_file.close();
    ROS_INFO("Data generation complete!");

    g_planning_scene.reset();
    g_current_robot_state.reset();
    return 0;
}
