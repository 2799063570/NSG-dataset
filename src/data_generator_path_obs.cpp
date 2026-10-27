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
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cstdlib>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <limits>
#include <algorithm>
#include <exception>
#include <filesystem>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*
    分析丢下过程：
    1. 生成全局随机障碍物：这个按照圆柱坐标系生成，不需要太难，要便于算法规划且不生成很简单的路径
    2. 采样起始状态和目标状态：保证两个状态有效，两个状态之间的距离不是很短，且不能之间无障碍物
    3. 设置规划器，设置规划器优化目标：路径长度 + 清晰度，规划路径，对规划的路径进行优化
    4. 在末端执行器连线上添加障碍物，保证末端执行器被阻挡
    5. 在路径周围添加障碍物，保证路径被阻挡
    6. 全局添加障碍物，增加规划的复杂度
    需要注意的是过程中需要将场景、机器人状态不断地更新到 rviz 中，便于调试和验证

    在 debug 为 false 的情况下
    节点只依赖于参数服务器中的 URDF 和 SRDF
*/

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = moveit_visual_tools;

// 障碍物类?
enum ShapeType { BOX = 0, SPHERE = 1, CYLINDER = 2 };

struct Obstacle {
    ShapeType type; 
    double x, y, z; 
    std::vector<double> dims; 
    Obstacle() : dims(3, 0.0) {} 
};

// 全局障碍物列表，用于路径凢?碰撞相关的距离计算与缓存
std::vector<Obstacle> g_all_scene_obstacles;

// 全局变量
planning_scene::PlanningScenePtr g_planning_scene;      
robot_state::RobotStatePtr g_current_robot_state;       
const robot_state::JointModelGroup* g_joint_model_group;
ros::Publisher g_collision_debug_pub;
bool g_collision_debug_enabled = false;

int findNextSceneId(const std::string& csv_path) {
    std::ifstream in(csv_path);
    if (!in.is_open()) {
        return 0;
    }

    int max_scene_id = -1;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string first_col;
        if (!std::getline(ss, first_col, ',')) continue;
        try {
            max_scene_id = std::max(max_scene_id, std::stoi(first_col));
        } catch (const std::exception&) {
            continue;
        }
    }
    return max_scene_id + 1;
}

void publishCollisionDebugMarker(const ob::State* state, std::size_t state_idx, const std::string& reason) {
    if (!g_collision_debug_enabled || !g_collision_debug_pub) return;

    const auto* joint_state = state->as<ob::RealVectorStateSpace::StateType>();
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, joint_state->values);
    g_current_robot_state->updateLinkTransforms();

    const std::string& ee_link = g_joint_model_group->getLinkModelNames().back();
    const Eigen::Vector3d ee_pos = g_current_robot_state->getGlobalLinkTransform(ee_link).translation();

    std::ostringstream oss;
    oss << reason << " at state " << state_idx << ", ee=("
        << ee_pos.x() << ", " << ee_pos.y() << ", " << ee_pos.z() << ") q=[";
    for (unsigned int i = 0; i < g_joint_model_group->getVariableCount(); ++i) {
        if (i > 0) oss << ", ";
        oss << joint_state->values[i];
    }
    oss << "]";
    ROS_WARN_STREAM(oss.str());

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = g_joint_model_group->getParentModel().getModelFrame();
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "collision_debug";
    sphere.id = 0;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.pose.position.x = ee_pos.x();
    sphere.pose.position.y = ee_pos.y();
    sphere.pose.position.z = ee_pos.z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.06;
    sphere.scale.y = 0.06;
    sphere.scale.z = 0.06;
    sphere.color.a = 0.95;
    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0;
    marker_array.markers.push_back(sphere);

    visualization_msgs::Marker text;
    text.header.frame_id = g_joint_model_group->getParentModel().getModelFrame();
    text.header.stamp = ros::Time::now();
    text.ns = "collision_debug";
    text.id = 1;
    text.action = visualization_msgs::Marker::ADD;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.pose.position.x = ee_pos.x();
    text.pose.position.y = ee_pos.y();
    text.pose.position.z = ee_pos.z() + 0.08;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.04;
    text.color.a = 1.0;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.text = reason + " state " + std::to_string(state_idx);
    marker_array.markers.push_back(text);

    g_collision_debug_pub.publish(marker_array);
}

// 清除 RViz 中所有碰撞调试标记（在新丢轮路径检查开始时调用?
void clearCollisionDebugMarkers() {
    if (!g_collision_debug_enabled || !g_collision_debug_pub) return;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    g_collision_debug_pub.publish(marker_array);
}

// ==========================================
// 自定义平滑：带碰撞检测的切角平滑
// ==========================================
void customSmoothPath(og::PathGeometric& path, const ob::SpaceInformationPtr& si, int max_iterations = 3) {
    if (path.getStateCount() < 3) return;
    ob::StateSpacePtr space = si->getStateSpace();
    ob::State* temp1 = space->allocState();
    ob::State* temp2 = space->allocState();

    for (int iter = 0; iter < max_iterations; ++iter) {
        bool changed = false;
        og::PathGeometric new_path(si);
        new_path.append(path.getState(0));

        for (size_t i = 1; i < path.getStateCount() - 1; ++i) {
            const ob::State* prev = path.getState(i-1);
            const ob::State* curr = path.getState(i);
            const ob::State* next = path.getState(i+1);

            space->interpolate(prev, curr, 0.7, temp1);
            space->interpolate(curr, next, 0.3, temp2);

            if (si->checkMotion(temp1, temp2)) {
                new_path.append(temp1); new_path.append(temp2);
                changed = true;
            } else {
                space->interpolate(prev, curr, 0.9, temp1);
                space->interpolate(curr, next, 0.1, temp2);
                if (si->checkMotion(temp1, temp2)) {
                    new_path.append(temp1); new_path.append(temp2);
                    changed = true;
                } else {
                    new_path.append(curr);
                }
            }
        }
        new_path.append(path.getState(path.getStateCount() - 1));
        path = new_path;
        if (!changed) break;
    }
    space->freeState(temp1); space->freeState(temp2);
}

// ==========================================
// 安全棢?
// ==========================================
bool isPathSafe(const og::PathGeometric& path, const ob::SpaceInformationPtr& si) {
    // 每次棢查前先清除上丢次的碰撞标记，防?无碰撞时旧标记残?
    clearCollisionDebugMarkers();
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        if (!si->isValid(path.getState(i)))
        {
            if (g_collision_debug_enabled)
            {
                ROS_WARN("collision at state %zu", i);
                publishCollisionDebugMarker(path.getState(i), i, "state collision");
            }
        
            return false;
        }
    }
    for (std::size_t i = 0; i < path.getStateCount() - 1; ++i) {
        if (!si->checkMotion(path.getState(i), path.getState(i+1))) {
            if (g_collision_debug_enabled)
                ROS_WARN("collision between states %zu and %zu", i, i+1);
            const std::size_t samples = 20;
            for (std::size_t s = 0; s <= samples; ++s) {
                const double alpha = static_cast<double>(s) / static_cast<double>(samples);
                ob::State* interp = si->getStateSpace()->allocState();
                si->getStateSpace()->interpolate(path.getState(i), path.getState(i+1), alpha, interp);
                if (!si->isValid(interp)) {
                    if (g_collision_debug_enabled)
                        publishCollisionDebugMarker(interp, i, "motion collision");
                    si->getStateSpace()->freeState(interp);
                    return false;
                }
                si->getStateSpace()->freeState(interp);
            }
            return false;
        }
    }
    return true;
}

// ==========================================
// 棢查路径与单个候障碍物的最小安全间?
// 采样路径中的关键状，对所有连杆做 FK，返回最小距?
// 若最小距?< min_clearance 则认为障碍物过近，拒绝添?
// ==========================================
double getObstacleDistToPath(const og::PathGeometric& path, const Obstacle& obs,
                              int sample_stride = 5) {
    double min_dist = std::numeric_limits<double>::max();
    const std::vector<std::string>& links = g_joint_model_group->getLinkModelNames();

    // 只采样部分路径状态，避免过慢（sample_stride 控制间隔?
    for (std::size_t i = 0; i < path.getStateCount(); i += sample_stride) {
        const auto* state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        g_current_robot_state->setJointGroupPositions(g_joint_model_group, state->values);
        g_current_robot_state->updateLinkTransforms();

        // 对每个连杆取中心点，计算到障碍物表面的距?
        for (const auto& link_name : links) {
            Eigen::Vector3d pt = g_current_robot_state->getGlobalLinkTransform(link_name).translation();
            double d = std::numeric_limits<double>::max();
            if (obs.type == BOX) {
                double dx = std::max(0.0, std::abs(pt.x() - obs.x) - obs.dims[0] / 2.0);
                double dy = std::max(0.0, std::abs(pt.y() - obs.y) - obs.dims[1] / 2.0);
                double dz = std::max(0.0, std::abs(pt.z() - obs.z) - obs.dims[2] / 2.0);
                d = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (obs.type == SPHERE) {
                double dx = pt.x() - obs.x, dy = pt.y() - obs.y, dz = pt.z() - obs.z;
                d = std::max(0.0, std::sqrt(dx*dx + dy*dy + dz*dz) - obs.dims[0]);
            } else { // CYLINDER
                double dx = pt.x() - obs.x, dy = pt.y() - obs.y;
                double r_dist = std::max(0.0, std::sqrt(dx*dx + dy*dy) - obs.dims[1]);
                double z_dist = std::max(0.0, std::abs(pt.z() - obs.z) - obs.dims[0] / 2.0);
                d = std::sqrt(r_dist*r_dist + z_dist*z_dist);
            }
            min_dist = std::min(min_dist, d);
        }
    }
    return min_dist;
}

double computeMaxReach(const robot_model::JointModelGroup* group) {
    double max_reach = 0.0;
    const std::vector<const robot_model::LinkModel*>& links = group->getLinkModels();
    for (const auto* link : links) {
        max_reach += link->getJointOriginTransform().translation().norm();
    }
    return max_reach;
}

double computeStateClearance(const ob::State* state, double safety_margin = 0.0) {
    const auto* joint_state = state->as<ob::RealVectorStateSpace::StateType>();
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, joint_state->values);
    g_current_robot_state->updateLinkTransforms();

    double min_dist = std::numeric_limits<double>::max();
    const std::vector<std::string>& links = g_joint_model_group->getLinkModelNames();

    for (const auto& link_name : links) {
        Eigen::Vector3d pt = g_current_robot_state->getGlobalLinkTransform(link_name).translation();
        for (const auto& obs : g_all_scene_obstacles) {
            double d = std::numeric_limits<double>::max();
            if (obs.type == BOX) {
                double dx = std::max(0.0, std::abs(pt.x() - obs.x) - obs.dims[0] / 2.0);
                double dy = std::max(0.0, std::abs(pt.y() - obs.y) - obs.dims[1] / 2.0);
                double dz = std::max(0.0, std::abs(pt.z() - obs.z) - obs.dims[2] / 2.0);
                d = std::sqrt(dx * dx + dy * dy + dz * dz);
            } else if (obs.type == SPHERE) {
                double dx = pt.x() - obs.x, dy = pt.y() - obs.y, dz = pt.z() - obs.z;
                d = std::max(0.0, std::sqrt(dx * dx + dy * dy + dz * dz) - obs.dims[0]);
            } else {
                double dx = pt.x() - obs.x, dy = pt.y() - obs.y;
                double r_dist = std::max(0.0, std::sqrt(dx * dx + dy * dy) - obs.dims[1]);
                double z_dist = std::max(0.0, std::abs(pt.z() - obs.z) - obs.dims[0] / 2.0);
                d = std::sqrt(r_dist * r_dist + z_dist * z_dist);
            }
            min_dist = std::min(min_dist, d);
        }
    }

    if (min_dist == std::numeric_limits<double>::max()) {
        min_dist = 0.0;
    }

    return std::max(0.0, min_dist - safety_margin);
}

class ClearancePenaltyObjective : public ob::OptimizationObjective {
public:
    ClearancePenaltyObjective(const ob::SpaceInformationPtr& si, double safety_margin, double epsilon)
        : ob::OptimizationObjective(si), safety_margin_(safety_margin), epsilon_(epsilon) {
        description_ = "Clearance penalty objective";
    }

    ob::Cost stateCost(const ob::State* s) const override {
        const double clearance = computeStateClearance(s, safety_margin_);
        // 褰掍竴鍖栨儵缃氾細clearance 瓒婂ぇ 鈫?penalty 瓒婂皬锛堢害 0~1 鑼冨洿锛?
        // ?clearance >> epsilon_ 时趋?0，当 clearance=0 时为 1.0
        const double penalty = epsilon_ / (clearance + epsilon_);
        return ob::Cost(penalty);
    }

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override {
        // 取两端点惩罚的平均，不乘?distance?
        // 这样路径长短不影响净空代价，规划器不会因绕远而被惩罚
        const double c1 = stateCost(s1).value();
        const double c2 = stateCost(s2).value();
        return ob::Cost(0.5 * (c1 + c2));
    }

    ob::Cost identityCost() const override {
        return ob::Cost(0.0);
    }

    ob::Cost infiniteCost() const override {
        return ob::Cost(std::numeric_limits<double>::infinity());
    }

private:
    double safety_margin_;
    double epsilon_;
};

bool isStateValid(const ob::State *state) {
    const auto *joint_state = state->as<ob::RealVectorStateSpace::StateType>();
    // 设置当前机器人状态并调用规划场景的碰撞检测接?
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, joint_state->values);
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.contacts = false; 
    g_planning_scene->checkCollision(c_req, c_res, *g_current_robot_state);
    return !c_res.collision;
}

// 将障碍物添加到场?
void addObstacleToScene(const std::string& id, const Obstacle& obs) {
    moveit_msgs::CollisionObject collision_object;
    // collision_object.header.frame_id = "base_link";
    collision_object.header.frame_id = g_joint_model_group->getParentModel().getModelFrame();
    collision_object.id = id;
    collision_object.operation = collision_object.ADD;

    shape_msgs::SolidPrimitive primitive;
    if (obs.type == BOX) {
        primitive.type = shape_msgs::SolidPrimitive::BOX;
        primitive.dimensions = {obs.dims[0], obs.dims[1], obs.dims[2]};
    } else if (obs.type == SPHERE) {
        primitive.type = shape_msgs::SolidPrimitive::SPHERE;
        primitive.dimensions = {obs.dims[0]};
    } else if (obs.type == CYLINDER) {
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive.dimensions = {obs.dims[0], obs.dims[1]};
    }
    
    geometry_msgs::Pose pose;
    pose.position.x = obs.x; pose.position.y = obs.y; pose.position.z = obs.z;
    pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    g_planning_scene->processCollisionObjectMsg(collision_object);
}
// 获取连杆在局部坐标系下的轴向包围?(AABB)
void getLinkLocalAABB(const moveit::core::LinkModel* link_model, 
                      Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) {
    // 初始化为极端?
    min_pt.setConstant(std::numeric_limits<double>::max());
    max_pt.setConstant(-std::numeric_limits<double>::max());

    if (!link_model) return;

    const std::vector<shapes::ShapeConstPtr>& shapes = link_model->getShapes();
    const auto& poses = link_model->getCollisionOriginTransforms();

    for (size_t i = 0; i < shapes.size(); ++i) {
        const auto& shape = shapes[i];
        const auto& pose = poses[i];

        if (shape->type == shapes::MESH) {
            auto mesh = static_cast<const shapes::Mesh*>(shape.get());
            for (unsigned int v = 0; v < mesh->vertex_count; ++v) {
                Eigen::Vector3d pt(mesh->vertices[3*v], mesh->vertices[3*v+1], mesh->vertices[3*v+2]);
                // 将顶点转换到连杆的局部坐标系
                Eigen::Vector3d local_pt = pose * pt; 
                // 鏇存柊鍖呭洿鐩掕竟鐣?
                min_pt = min_pt.cwiseMin(local_pt);
                max_pt = max_pt.cwiseMax(local_pt);
            }
        } else if (shape->type == shapes::CYLINDER) {
            auto cylinder = static_cast<const shapes::Cylinder*>(shape.get());
            double r = cylinder->radius;
            double h = cylinder->length / 2.0;
            // 箢单估算圆柱体?8 个顶?
            std::vector<Eigen::Vector3d> corners = {
                {r, r, h}, {r, -r, h}, {-r, r, h}, {-r, -r, h},
                {r, r, -h}, {r, -r, -h}, {-r, r, -h}, {-r, -r, -h}
            };
            for (const auto& corner : corners) {
                Eigen::Vector3d local_pt = pose * corner;
                min_pt = min_pt.cwiseMin(local_pt);
                max_pt = max_pt.cwiseMax(local_pt);
            }
        } else if (shape->type == shapes::BOX) {
            auto box = static_cast<const shapes::Box*>(shape.get());
            double lx = box->size[0] / 2.0, ly = box->size[1] / 2.0, lz = box->size[2] / 2.0;
            std::vector<Eigen::Vector3d> corners = {
                {lx, ly, lz}, {lx, -ly, lz}, {-lx, ly, lz}, {-lx, -ly, lz},
                {lx, ly, -lz}, {lx, -ly, -lz}, {-lx, ly, -lz}, {-lx, -ly, -lz}
            };
            for (const auto& corner : corners) {
                Eigen::Vector3d local_pt = pose * corner;
                min_pt = min_pt.cwiseMin(local_pt);
                max_pt = max_pt.cwiseMax(local_pt);
            }
        } else if (shape->type == shapes::SPHERE) {
            auto sphere = static_cast<const shapes::Sphere*>(shape.get());
            double r = sphere->radius;
            std::vector<Eigen::Vector3d> corners = {
                {r, r, r}, {r, -r, r}, {-r, r, r}, {-r, -r, r},
                {r, r, -r}, {r, -r, -r}, {-r, r, -r}, {-r, -r, -r}
            };
            for (const auto& corner : corners) {
                Eigen::Vector3d local_pt = pose * corner;
                min_pt = min_pt.cwiseMin(local_pt);
                max_pt = max_pt.cwiseMax(local_pt);
            }
        }
    }
    
    // 如果没有获取到形状（例如虚拟连杆），给一个默认的微小盒子
    if (min_pt.x() > max_pt.x()) {
        min_pt.setConstant(-0.01);
        max_pt.setConstant(0.01);
    }
}
void removeObstacleFromScene(const std::string& id) {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = g_joint_model_group->getParentModel().getModelFrame();
    collision_object.id = id;
    collision_object.operation = collision_object.REMOVE;
    g_planning_scene->processCollisionObjectMsg(collision_object);
}

Obstacle makeBoxObstacle(double x, double y, double z, double sx, double sy, double sz) {
    Obstacle obs;
    obs.type = BOX;
    obs.x = x;
    obs.y = y;
    obs.z = z;
    obs.dims = {sx, sy, sz};
    return obs;
}

Obstacle makeCylinderObstacle(double x, double y, double z, double height, double radius) {
    Obstacle obs;
    obs.type = CYLINDER;
    obs.x = x;
    obs.y = y;
    obs.z = z;
    obs.dims = {height, radius, 0.0};
    return obs;
}

std::vector<Obstacle> makeShelfApplicationScene(double cx, double cy, double cz,
                                                double width, double depth,
                                                double height, double board_thickness,
                                                bool add_table, bool add_back_board,
                                                int box_count) {
    std::vector<Obstacle> obstacles;

    if (add_table) {
        obstacles.push_back(makeCylinderObstacle(0.0, 0.0, -0.01, 0.02, 1.0));
    }

    const double half_width = width * 0.5;
    const double half_depth = depth * 0.5;
    const double half_height = height * 0.5;

    // The shelf faces +X. It is represented by multiple boxes so the existing
    // CSV and MoveIt collision pipeline can consume it without new semantics.
    obstacles.push_back(makeBoxObstacle(cx, cy - half_width, cz, depth, board_thickness, height));
    obstacles.push_back(makeBoxObstacle(cx, cy + half_width, cz, depth, board_thickness, height));
    obstacles.push_back(makeBoxObstacle(cx, cy, cz - half_height, depth, width, board_thickness));
    obstacles.push_back(makeBoxObstacle(cx, cy, cz + half_height, depth, width, board_thickness));
    obstacles.push_back(makeBoxObstacle(cx, cy, cz, depth, width, board_thickness));

    if (add_back_board) {
        obstacles.push_back(makeBoxObstacle(cx - half_depth, cy, cz, board_thickness, width, height));
    }

    const double box_sx = std::min(0.16, depth * 0.55);
    const double box_sy = std::min(0.18, width * 0.22);
    const double box_sz = std::min(0.12, height * 0.18);
    const double clearance = 0.01;

    const std::vector<double> level_z = {
        cz - half_height + board_thickness + box_sz * 0.5 + clearance,
        cz + board_thickness + box_sz * 0.5 + clearance
    };
    const std::vector<double> slot_y = {
        cy - width * 0.25,
        cy,
        cy + width * 0.25
    };

    const int clamped_box_count = std::max(0, std::min(box_count, 6));
    for (int i = 0; i < clamped_box_count; ++i) {
        const double z = level_z[static_cast<size_t>(i / 3)];
        const double y = slot_y[static_cast<size_t>(i % 3)];
        obstacles.push_back(makeBoxObstacle(cx + depth * 0.15, y, z, box_sx, box_sy, box_sz));
    }

    return obstacles;
}

std::vector<Obstacle> makeTableBoxApplicationScene(double cx, double cy,
                                                   double table_top_z,
                                                   double table_size_x,
                                                   double table_size_y,
                                                   double table_thickness,
                                                   double box_height,
                                                   int box_count,
                                                   bool add_back_guard,
                                                   bool add_side_guards) {
    std::vector<Obstacle> obstacles;

    const double clamped_table_size_x = std::max(0.20, table_size_x);
    const double clamped_table_size_y = std::max(0.20, table_size_y);
    const double clamped_table_thickness = std::max(0.01, table_thickness);
    const double clamped_box_height = std::max(0.04, box_height);

    // A thin table top provides a realistic placing surface while preserving
    // the existing obstacle-only CSV representation.
    obstacles.push_back(makeBoxObstacle(cx, cy, table_top_z - clamped_table_thickness * 0.5,
                                        clamped_table_size_x, clamped_table_size_y,
                                        clamped_table_thickness));

    const double guard_thickness = 0.035;
    const double guard_height = 0.18;
    const double guard_z = table_top_z + guard_height * 0.5;
    if (add_back_guard) {
        obstacles.push_back(makeBoxObstacle(cx - clamped_table_size_x * 0.5 + guard_thickness * 0.5,
                                            cy, guard_z, guard_thickness,
                                            clamped_table_size_y, guard_height));
    }
    if (add_side_guards) {
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
        {cx - clamped_table_size_x * 0.18, cy - clamped_table_size_y * 0.18}
    };

    const int clamped_box_count = std::max(0, std::min(box_count, static_cast<int>(slots.size())));
    for (int i = 0; i < clamped_box_count; ++i) {
        obstacles.push_back(makeBoxObstacle(slots[static_cast<size_t>(i)].first,
                                            slots[static_cast<size_t>(i)].second,
                                            object_z, box_sx, box_sy,
                                            clamped_box_height));
    }

    return obstacles;
}

visualization_msgs::Marker makeObstacleMarker(const Obstacle& obs, const std::string& frame_id,
                                              const std::string& ns, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obs.x;
    marker.pose.position.y = obs.y;
    marker.pose.position.z = obs.z;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 0.85;

    if (obs.type == BOX) {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = obs.dims[0];
        marker.scale.y = obs.dims[1];
        marker.scale.z = obs.dims[2];
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else if (obs.type == SPHERE) {
        marker.type = visualization_msgs::Marker::SPHERE;
        double diameter = obs.dims[0] * 2.0;
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = diameter;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
    } else {
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = obs.dims[1] * 2.0;
        marker.scale.y = obs.dims[1] * 2.0;
        marker.scale.z = obs.dims[0];
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    return marker;
}

void clearObstaclePreview(ros::Publisher& marker_pub) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker_pub.publish(marker_array);
}

void publishObstaclePreview(ros::Publisher& marker_pub, const Obstacle& obs, const std::string& frame_id,
                            const std::string& ns, int id) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(makeObstacleMarker(obs, frame_id, ns, id));
    marker_pub.publish(marker_array);
}

// ==========================================
// ?障碍物碰撞检测（?Python 验证器一致）
// ==========================================
bool checkPointInObstacles(const Eigen::Vector3d& pt, const std::vector<Obstacle>& obstacles, double safety_margin = 0.005) {
    for (const auto& obs : obstacles) {
        if (obs.type == BOX) {
            if (std::abs(pt.x() - obs.x) <= obs.dims[0] / 2.0 + safety_margin &&
                std::abs(pt.y() - obs.y) <= obs.dims[1] / 2.0 + safety_margin &&
                std::abs(pt.z() - obs.z) <= obs.dims[2] / 2.0 + safety_margin) {
                return true;
            }
        } else if (obs.type == SPHERE) {
            double dx = pt.x() - obs.x, dy = pt.y() - obs.y, dz = pt.z() - obs.z;
            if (dx*dx + dy*dy + dz*dz <= (obs.dims[0] + safety_margin) * (obs.dims[0] + safety_margin)) {
                return true;
            }
        } else if (obs.type == CYLINDER) {
            double dx = pt.x() - obs.x, dy = pt.y() - obs.y;
            double r = obs.dims[1]; // 半径
            double h = obs.dims[0]; // 楂樺害
            if (dx*dx + dy*dy <= (r + safety_margin) * (r + safety_margin) &&
                std::abs(pt.z() - obs.z) <= h / 2.0 + safety_margin) {
                return true;
            }
        }
    }
    return false;
}

// ==========================================
// 末端执行器直线碰撞检测（?Python 验证器辑丢致）
// 在关节空间起止点间线性插值，FK 求末端位置，棢测是否碰障碍?
// ==========================================
bool checkEELineCollision(const ob::State* start_state, const ob::State* goal_state,
                          const std::vector<Obstacle>& obstacles, int interp_steps = 12) {
    const auto* s = start_state->as<ob::RealVectorStateSpace::StateType>();
    const auto* g = goal_state->as<ob::RealVectorStateSpace::StateType>();
    int dim = g_joint_model_group->getVariableCount();
    const std::string& ee_link = g_joint_model_group->getLinkModelNames().back();

    for (int i = 0; i < interp_steps; ++i) {
        double alpha = static_cast<double>(i) / (interp_steps - 1);
        std::vector<double> q(dim);
        for (int j = 0; j < dim; ++j) {
            q[j] = (1.0 - alpha) * s->values[j] + alpha * g->values[j];
        }
        g_current_robot_state->setJointGroupPositions(g_joint_model_group, q.data());
        g_current_robot_state->updateLinkTransforms();
        Eigen::Vector3d ee_pos = g_current_robot_state->getGlobalLinkTransform(ee_link).translation();

        if (checkPointInObstacles(ee_pos, obstacles, 0.0)) {
            return true; // 末端执行器直线碰到障碍物
        }
    }
    return false;
}

// ==========================================
// 在末端执行器起止点连线上放置遮挡障碍?
// ==========================================
Obstacle generateEEBlockingObstacle(const ob::State* start_state, const ob::State* goal_state,
                                     double min_size = 0.05, double max_size = 0.2) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    const auto* s = start_state->as<ob::RealVectorStateSpace::StateType>();
    const auto* g = goal_state->as<ob::RealVectorStateSpace::StateType>();
    const std::string& ee_link = g_joint_model_group->getLinkModelNames().back();

    // 计算起止点末端执行器位置
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, s->values);
    g_current_robot_state->updateLinkTransforms();
    Eigen::Vector3d ee_start = g_current_robot_state->getGlobalLinkTransform(ee_link).translation();

    g_current_robot_state->setJointGroupPositions(g_joint_model_group, g->values);
    g_current_robot_state->updateLinkTransforms();
    Eigen::Vector3d ee_goal = g_current_robot_state->getGlobalLinkTransform(ee_link).translation();

    // 在连线的 25%~75% 处随机一个点
    std::uniform_real_distribution<> t_dist(0.25, 0.75);
    std::uniform_real_distribution<> offset_dist(-0.05, 0.05);
    std::uniform_real_distribution<> size_dist(min_size, max_size);
    std::uniform_int_distribution<> type_dist(0, 2);

    double t = t_dist(gen);
    Eigen::Vector3d center = ee_start + t * (ee_goal - ee_start);
    // 轻微偏移，让规划更自?
    center.x() += offset_dist(gen);
    center.y() += offset_dist(gen);
    center.z() += offset_dist(gen);

    Obstacle obs;
    obs.type = static_cast<ShapeType>(type_dist(gen));
    obs.x = center.x();
    obs.y = center.y();
    obs.z = center.z();

    if (obs.type == BOX) {
        obs.dims[0] = size_dist(gen); obs.dims[1] = size_dist(gen); obs.dims[2] = size_dist(gen);
    } else if (obs.type == SPHERE) {
        obs.dims[0] = size_dist(gen) * 0.6;
    } else { // CYLINDER
        obs.dims[0] = size_dist(gen);       // 楂樺害
        obs.dims[1] = size_dist(gen) * 0.5; // 半径
    }
    return obs;
}

// 阶段丢：全屢障碍?
// 生成全局工作空间内的随机障碍?
// 鍙傛暟璇存槑锛?
// min_radius: 朢小半?(比如 0.2m，为了防止障碍物直接生成在机器人底座中心发生碰撞)
// max_radius: 朢大半?(根据 AUBO i5 的臂展，通常设置?0.8m - 1.0m 左右)
// min_z, max_z: 高度范围 (比如 -0.1m ?1.2m)
Obstacle generateGlobalWorkspaceObstacle(double min_radius = 0.2, 
                                         double max_radius = 1.0, 
                                         double min_z = -0.1, 
                                         double max_z = 1.2) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // 全局障碍物的尺寸调大，以起到封堵空间的作?
    std::uniform_real_distribution<> size_dist(0.15, 0.45); 
    std::uniform_int_distribution<> type_dist(0, 2);

    Obstacle obs;
    obs.type = static_cast<ShapeType>(type_dist(gen));

    // 1. 生成障碍物尺?
    if (obs.type == BOX) {
        obs.dims[0] = size_dist(gen); obs.dims[1] = size_dist(gen); obs.dims[2] = size_dist(gen);
    } else if (obs.type == SPHERE) {
        obs.dims[0] = size_dist(gen) * 0.5;
    } else { // CYLINDER
        obs.dims[0] = size_dist(gen); // 楂樺害
        obs.dims[1] = size_dist(gen) * 0.4; // 半径
    }

    // ================= 2. 圆柱环形空间均匀采样 =================
    std::uniform_real_distribution<> u_dist(0.0, 1.0);
    std::uniform_real_distribution<> theta_dist(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> z_dist(min_z, max_z);

    // 关键数学逻辑：要在圆环面积内均匀分布，半径必须开根号?
    // 否则障碍物会密集扎堆在中心点附近?
    double u = u_dist(gen);
    double r = std::sqrt(u * (max_radius * max_radius - min_radius * min_radius) + min_radius * min_radius);
    
    double theta = theta_dist(gen);

    // 3. 鏋佸潗鏍囪浆绗涘崱灏斿潗鏍?
    obs.x = r * std::cos(theta);
    obs.y = r * std::sin(theta);
    obs.z = z_dist(gen);

    return obs;
}

// 阶段二：贴脸逆向生成
Obstacle generateNearPathObstacle(const og::PathGeometric& path, 
                                  double target_clearance = 0.04,
                                  double random_range = 0.1) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> size_dist(0.05, 0.2);// 障碍物尺寸范?
    std::uniform_int_distribution<> type_dist(0, 2);// 障碍物类?

    // 1. 鑾峰彇鏈哄櫒浜鸿繛鏉嗕綅濮?
    int idx = rand() % path.getStateCount();// 从路径中随机选一个状?
    const auto* state = path.getState(idx)->as<ob::RealVectorStateSpace::StateType>();
    g_current_robot_state->setJointGroupPositions(g_joint_model_group, state->values);
    g_current_robot_state->updateLinkTransforms();

    const std::vector<std::string>& links = g_joint_model_group->getLinkModelNames();
    if (links.empty()) {
        Obstacle fallback;
        fallback.type = static_cast<ShapeType>(type_dist(gen));
        return fallback;
    }

    // 判断link数量，避免下标越?
    size_t link_idx = 0;
    if (links.size() > 4) {
        link_idx = 4 + static_cast<size_t>(rand()) % (links.size() - 4);
    } else {
        link_idx = static_cast<size_t>(rand()) % links.size();
    }
    // const Eigen::Isometry3d& link_pose = g_current_robot_state->getGlobalLinkTransform(links[link_idx]);

    Obstacle obs;
    obs.type = static_cast<ShapeType>(type_dist(gen));

    // 2. 生成障碍物尺寸，并计算外接包围球半径
    double obs_radius = 0.0;
    if (obs.type == BOX) {
        obs.dims[0] = size_dist(gen); obs.dims[1] = size_dist(gen); obs.dims[2] = size_dist(gen);
        obs_radius = 0.5 * std::sqrt(obs.dims[0]*obs.dims[0] + obs.dims[1]*obs.dims[1] + obs.dims[2]*obs.dims[2]);
    } else if (obs.type == SPHERE) {
        obs.dims[0] = size_dist(gen) * 0.5;
        obs_radius = obs.dims[0];
    } else { 
        obs.dims[0] = size_dist(gen); obs.dims[1] = size_dist(gen) * 0.4;
        obs_radius = std::sqrt(obs.dims[1]*obs.dims[1] + (obs.dims[0]/2.0)*(obs.dims[0]/2.0));
    }

    // ================= 3. 获取连杆的局部包围盒并向外扩?=================
    const std::string& selected_link_name = links[link_idx];
    const Eigen::Isometry3d& link_pose = g_current_robot_state->getGlobalLinkTransform(selected_link_name);

    const moveit::core::LinkModel* link_model = g_joint_model_group->getLinkModel(selected_link_name);
    Eigen::Vector3d min_pt, max_pt;
    getLinkLocalAABB(link_model, min_pt, max_pt);

    // 扩张?= 障碍物自身半?+ 期望的安全净?+ 随机浮动
    double expansion = obs_radius + target_clearance + std::uniform_real_distribution<>(0.0, random_range)(gen);
    
    // 得到扩张后的安全包围?
    Eigen::Vector3d safe_min = min_pt.array() - expansion;
    Eigen::Vector3d safe_max = max_pt.array() + expansion;

    // ================= 4. 在扩张后的长方体表面随机采样丢个点 =================
    // 随机选择长方体的 6 个面之一 (X? Y? 还是 Z?
    int face = rand() % 6; 
    Eigen::Vector3d local_target_pt;

    // 辅助生成范围内随机数?lambda
    auto rand_in_range = [&](double min_val, double max_val) {
        return std::uniform_real_distribution<>(min_val, max_val)(gen);
    };

    if (face == 0) { // 贴在朢?X ?
        local_target_pt << safe_min.x(), rand_in_range(safe_min.y(), safe_max.y()), rand_in_range(safe_min.z(), safe_max.z());
    } else if (face == 1) { // 贴在朢?X ?
        local_target_pt << safe_max.x(), rand_in_range(safe_min.y(), safe_max.y()), rand_in_range(safe_min.z(), safe_max.z());
    } else if (face == 2) { // 贴在朢?Y ?
        local_target_pt << rand_in_range(safe_min.x(), safe_max.x()), safe_min.y(), rand_in_range(safe_min.z(), safe_max.z());
    } else if (face == 3) { // 贴在朢?Y ?
        local_target_pt << rand_in_range(safe_min.x(), safe_max.x()), safe_max.y(), rand_in_range(safe_min.z(), safe_max.z());
    } else if (face == 4) { // 贴在朢?Z ?
        local_target_pt << rand_in_range(safe_min.x(), safe_max.x()), rand_in_range(safe_min.y(), safe_max.y()), safe_min.z();
    } else { // 贴在朢?Z ?
        local_target_pt << rand_in_range(safe_min.x(), safe_max.x()), rand_in_range(safe_min.y(), safe_max.y()), safe_max.z();
    }

    // ================= 5. 将局部采样点转换到全屢坐标?=================
    Eigen::Vector3d global_target_pt = link_pose * local_target_pt;

    obs.x = global_target_pt.x();
    obs.y = global_target_pt.y();
    obs.z = global_target_pt.z();

    return obs;
}

void getRobotConf(ob::RealVectorBounds& bounds, int dim) {
    const robot_model::RobotModel& robot_model = g_joint_model_group->getParentModel();
    const std::vector<std::string>& vars = g_joint_model_group->getVariableNames();
    for (unsigned int i = 0; i < dim; ++i) {
        const auto& b = robot_model.getVariableBounds(vars[i]);
        if (b.position_bounded_) {
            bounds.setLow(i, b.min_position_); bounds.setHigh(i, b.max_position_);
        } else {
            bounds.setLow(i, -M_PI); bounds.setHigh(i, M_PI);
        }
    }
}
// 对路径离散点 离散点存储到列表中，并发布到 RViz 中显示路?
void visualizePath(og::PathGeometric& path, moveit_visual_tools::MoveItVisualToolsPtr& visual_tools) {
    if (!visual_tools) return;
    std::vector<geometry_msgs::Point> points;
    const std::string& ee_link = g_joint_model_group->getLinkModelNames().back();
    for (size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        g_current_robot_state->setJointGroupPositions(g_joint_model_group, s->values);
        g_current_robot_state->updateLinkTransforms();
        const Eigen::Isometry3d& pose = g_current_robot_state->getGlobalLinkTransform(ee_link);
        geometry_msgs::Point p; p.x = pose.translation().x(); p.y = pose.translation().y(); p.z = pose.translation().z();
        points.push_back(p);
    }
    visual_tools->publishPath(points, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    visual_tools->trigger();
}

void publishPlanningScene(ros::Publisher& scene_pub) {
    moveit_msgs::PlanningScene scene_msg;
    g_planning_scene->getPlanningSceneMsg(scene_msg);
    scene_msg.is_diff = false;
    scene_pub.publish(scene_msg);
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ompl_path_obs_generator");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    bool debug = false;
    // 参数说明：规划前场景obs-num 规划路劲附近添加obs-num 规划的路径数 每条路径朢少附近添加的obs-num 额外添加的全屢obs-num
    int init_obs = 3, add_obs = 10, num_base_paths = 1000, min_planted = 6, extra_global_obs_count = 6;
    int ee_blocking_obs_count = 2; // EE连线添加的obs-num
    int variants_per_path = 5; // 每条路径衍生出几个不同的极限场景
    double planner_clearance_weight = 0.45; // 规划器安全目标中凢空惩罚的权重?~1），数越??越向于远离障碍物的路?
    double planner_clearance_margin = 0.01; // 规划器安全目标中凢空惩罚的边界（单位：米），当路径点与障碍物距离小于该值时，开始产生明显的惩罚
    double planner_clearance_epsilon = 0.03; // 规划器安全目标中凢空惩罚的平滑项（单位：米），防止数过大导致优化器不稳定，建议设置?margin ?1/3 ~ 1/2
    bool collision_debug = false; // 是否打印碰撞信息并发?RViz 碰撞标记
    std::string application_scene_type = "random";
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
    p_nh.param("debug", debug, debug);
    p_nh.param("init_obs", init_obs, init_obs);
    p_nh.param("add_obs", add_obs, add_obs);
    p_nh.param("num_base_paths", num_base_paths, num_base_paths);
    p_nh.param("min_planted", min_planted, min_planted);
    p_nh.param("extra_global_obs_count", extra_global_obs_count, extra_global_obs_count);
    p_nh.param("ee_blocking_obs", ee_blocking_obs_count, ee_blocking_obs_count);
    p_nh.param("variants_per_path", variants_per_path, variants_per_path);
    p_nh.param("planner_clearance_weight", planner_clearance_weight, planner_clearance_weight);
    p_nh.param("planner_clearance_margin", planner_clearance_margin, planner_clearance_margin);
    p_nh.param("planner_clearance_epsilon", planner_clearance_epsilon, planner_clearance_epsilon);
    p_nh.param("collision_debug", collision_debug, collision_debug);
    p_nh.param("application_scene_type", application_scene_type, application_scene_type);
    p_nh.param("shelf_center_x", shelf_center_x, shelf_center_x);
    p_nh.param("shelf_center_y", shelf_center_y, shelf_center_y);
    p_nh.param("shelf_center_z", shelf_center_z, shelf_center_z);
    p_nh.param("shelf_width", shelf_width, shelf_width);
    p_nh.param("shelf_depth", shelf_depth, shelf_depth);
    p_nh.param("shelf_height", shelf_height, shelf_height);
    p_nh.param("shelf_board_thickness", shelf_board_thickness, shelf_board_thickness);
    p_nh.param("shelf_add_table", shelf_add_table, shelf_add_table);
    p_nh.param("shelf_add_back_board", shelf_add_back_board, shelf_add_back_board);
    p_nh.param("shelf_box_count", shelf_box_count, shelf_box_count);
    p_nh.param("table_center_x", table_center_x, table_center_x);
    p_nh.param("table_center_y", table_center_y, table_center_y);
    p_nh.param("table_top_z", table_top_z, table_top_z);
    p_nh.param("table_size_x", table_size_x, table_size_x);
    p_nh.param("table_size_y", table_size_y, table_size_y);
    p_nh.param("table_thickness", table_thickness, table_thickness);
    p_nh.param("table_box_height", table_box_height, table_box_height);
    p_nh.param("table_box_count", table_box_count, table_box_count);
    p_nh.param("table_add_back_guard", table_add_back_guard, table_add_back_guard);
    p_nh.param("table_add_side_guards", table_add_side_guards, table_add_side_guards);
    g_collision_debug_enabled = debug && collision_debug;
    ROS_INFO("Parameters: debug=%s, init_obs=%d, add_obs=%d, num_base_paths=%d, min_planted=%d, extra_global=%d, ee_blocking=%d, variants_per_path=%d", 
             debug ? "true" : "false", init_obs, add_obs, num_base_paths, min_planted, extra_global_obs_count, ee_blocking_obs_count, variants_per_path);
    ROS_INFO("Application scene: type=%s, shelf_center=(%.3f, %.3f, %.3f), shelf_size=(width %.3f, depth %.3f, height %.3f), shelf_boxes=%d",
             application_scene_type.c_str(), shelf_center_x, shelf_center_y, shelf_center_z,
             shelf_width, shelf_depth, shelf_height, shelf_box_count);
    ROS_INFO("Table-box scene: table_center=(%.3f, %.3f), table_top_z=%.3f, table_size=(%.3f, %.3f), table_boxes=%d",
             table_center_x, table_center_y, table_top_z, table_size_x, table_size_y, table_box_count);
    ROS_INFO("Collision debug: %s", g_collision_debug_enabled ? "enabled" : "disabled");
    ROS_INFO("Planner safety objective: clearance_weight=%.3f, clearance_margin=%.3f, clearance_epsilon=%.3f",
             planner_clearance_weight, planner_clearance_margin, planner_clearance_epsilon);

    ros::AsyncSpinner spinner(1); spinner.start();

    robot_model_loader::RobotModelLoader loader("robot_description");
    auto k_model = loader.getModel();// robotModel
    if (!k_model) {
        ROS_ERROR("Failed to load robot model.");
        return -1;
    }

    g_planning_scene.reset(new planning_scene::PlanningScene(k_model));
    g_current_robot_state.reset(new robot_state::RobotState(k_model));
    
    std::string group_name = "arm"; 
    g_joint_model_group = g_current_robot_state->getJointModelGroup(group_name);
    if (!g_joint_model_group) {
        ROS_ERROR("Joint group '%s' not found!", group_name.c_str());
        return -1;
    }

    rvt::MoveItVisualToolsPtr visual_tools;
    ros::Publisher scene_pub;
    ros::Publisher start_state_pub;
    ros::Publisher goal_state_pub;
    // preview_generated_obstacles 独立于 debug，可单独开启障碍物预览
    // 当 debug=true && preview_generated_obstacles=true 时，每个候选障碍物均会在 RViz 中预览
    // RViz 面板按钮说明：
    //   [Next]     预览下一个障碍物
    //   [Continue] 跳过当前阶段剩余的所有障碍物预览，直接完成本轮植入
    bool preview_generated_obstacles = true;
    p_nh.param("preview_generated_obstacles", preview_generated_obstacles, preview_generated_obstacles);
    
    if (debug) {
        // 初始化发布器
        scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);// 发布规划场景（障碍物）
        g_collision_debug_pub = nh.advertise<visualization_msgs::MarkerArray>("collision_debug_markers", 10, true);// 发布碰撞的路径点
        start_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_start_state", 10, true);
        goal_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_goal_state", 10, true);
        // 初始化工作流相关可视化控件
        visual_tools.reset(new rvt::MoveItVisualTools(k_model->getModelFrame(), "/rviz_visual_tools", k_model));
        visual_tools->loadRemoteControl(); visual_tools->deleteAllMarkers();
        ros::Duration(1.0).sleep();
        ROS_INFO("Collision debug topic: /collision_debug_markers");
        if (preview_generated_obstacles) {
            ROS_INFO("Obstacle preview ENABLED. RViz buttons: [Next]=preview next, [Continue]=skip remaining previews");
        }
    }

    auto space(std::make_shared<ob::RealVectorStateSpace>(g_joint_model_group->getVariableCount()));// 读取机器人维?设置关节空间
    ob::RealVectorBounds bounds(g_joint_model_group->getVariableCount());
    getRobotConf(bounds, g_joint_model_group->getVariableCount());// 读取关节限位
    space->setBounds(bounds);// 设置关节限位

    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(isStateValid); si->setup();// 设置状有效检查器(调用碰撞棢?

    double max_r = computeMaxReach(g_joint_model_group);// 计算机器人最大臂展，用于指导障碍物生成的半径范围
    
    const char* home = std::getenv("HOME");
    std::string output_dir = std::string(home) + "/aubo/arm_ws/src/motion_planning/data/";
    bool overwrite_output = false;
    p_nh.param("output_dir", output_dir, output_dir);
    p_nh.param("overwrite_output", overwrite_output, overwrite_output);
    if (!output_dir.empty() && output_dir.back() != '/') {
        output_dir += "/";
    }

    std::string prefix = output_dir;
    try {
        std::filesystem::create_directories(prefix);
    } catch (const std::filesystem::filesystem_error& e) {
        ROS_ERROR("Failed to create output directory %s: %s", prefix.c_str(), e.what());
        return -1;
    }

    const std::string env_path = prefix + "training_envs.csv";
    const std::string path_path = prefix + "training_paths.csv";
    if (overwrite_output) {
        std::ofstream(env_path, std::ios::trunc).close();
        std::ofstream(path_path, std::ios::trunc).close();
        ROS_WARN("overwrite_output=true, truncated existing dataset files in %s", prefix.c_str());
    }
    int global_scene_id = std::max(findNextSceneId(env_path), findNextSceneId(path_path));
    ROS_INFO("Starting scene_id from %d", global_scene_id);

    std::ofstream env_file(env_path, std::ios::app);
    std::ofstream path_file(path_path, std::ios::app);

    if (!env_file.is_open() || !path_file.is_open()) {
        ROS_ERROR("Failed to open data files! Make sure directory exists: %s", prefix.c_str());
        return -1;
    }

    // global_scene_id is initialized from existing CSV files to keep append runs unique.

    // 为了显示文字的位?(机器人上?
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    for (int base_idx = 0; base_idx < num_base_paths; ++base_idx) {    
        if (!ros::ok()) break;

        g_planning_scene->removeAllCollisionObjects();// 清理场景中的扢有障碍物
        g_all_scene_obstacles.clear();// 清理全局障碍物列?
        if (debug) {
            publishPlanningScene(scene_pub);// 清理rviz中的障碍物显?
        }
        std::vector<Obstacle> base_obstacles;// 存放桌子 ?第一阶段障碍?

        if (application_scene_type == "shelf") {
            base_obstacles = makeShelfApplicationScene(shelf_center_x, shelf_center_y, shelf_center_z,
                                                       shelf_width, shelf_depth, shelf_height,
                                                       shelf_board_thickness, shelf_add_table,
                                                       shelf_add_back_board, shelf_box_count);
        } else if (application_scene_type == "table_box" || application_scene_type == "table_place") {
            base_obstacles = makeTableBoxApplicationScene(table_center_x, table_center_y, table_top_z,
                                                          table_size_x, table_size_y, table_thickness,
                                                          table_box_height, table_box_count,
                                                          table_add_back_guard, table_add_side_guards);
        } else {
            // 鍩虹妗岄潰
            Obstacle table = makeCylinderObstacle(0.0, 0.0, -0.01, 0.02, 1.0);
            base_obstacles.push_back(table);

            // ================================================================
            // 阶段0：放置少量简单全屢障碍?
            // 目标：让关节空间直线不可通行（路径非平凡），不追求复杂?
            // ================================================================
            for (int i = 0; i < init_obs; ++i) {
                // 生成丢个全屢障碍?输入: 圆柱坐标系半径高度范?
                Obstacle ob = generateGlobalWorkspaceObstacle(max_r*0.3, max_r*0.85, -0.05, 1.1);
                base_obstacles.push_back(ob);
            }
        }

        for (size_t i = 0; i < base_obstacles.size(); ++i) {
            g_all_scene_obstacles.push_back(base_obstacles[i]);
            addObstacleToScene("base_obs_" + std::to_string(i), base_obstacles[i]);
        }

        // ================================================================
        // 閲囨牱璧锋鐐癸細
        //   (1) 璧锋鐐规湁鏁?(2) 璺濈澶熻繙
        //   (3) 全身碰撞棢测：关节空间直线必须碰撞（si->checkMotion 棢查所有连杆）
        // 注意：此阶段环境箢单，采样成功率高
        // ================================================================
        ob::ScopedState<> start(space), goal(space);
        bool valid = false; int attempts = 0;
        int max_attempts = 3000;
        while (!valid && attempts++ < max_attempts) {
            start.random(); goal.random();
            if (isStateValid(start.get()) && isStateValid(goal.get())) {
                if (space->distance(start.get(), goal.get()) > 1.0) {
                    // 核心：全身碰撞预棢 ?只有关节空间直线不可通行时才接受
                    // si->checkMotion() 棢查整条机械臂扢有连杆，不仅仅是末端
                    if (!si->checkMotion(start.get(), goal.get())) {
                        valid = true;
                    }
                }
            }
        }
        if (!valid) { 
            ROS_WARN("Failed to find detoured start/goal after %d attempts. Skipping base_idx %d", max_attempts, base_idx);
            base_idx--;
            continue; 
        }
        ROS_INFO("Base path %d: start/goal found after %d attempts (full-body collision 鉁?", base_idx, attempts);
        
        // 【Debug】显示基硢场景
        if (debug) {
            publishPlanningScene(scene_pub);// 发布当前场景?RViz

            visual_tools->deleteAllMarkers(); // Rvizg中更新当前的路径ID
            visual_tools->publishText(text_pose, "Base Path " + std::to_string(base_idx), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            visual_tools->trigger();
            
            // 显示起点和终?
            const std::string& ee_link = g_joint_model_group->getLinkModelNames().back();
            
            // 鍙戝竷璧风偣鏈鍧愭爣绯?
            g_current_robot_state->setJointGroupPositions(g_joint_model_group, start->as<ob::RealVectorStateSpace::StateType>()->values);
            g_current_robot_state->updateLinkTransforms();
            const Eigen::Isometry3d start_pose = g_current_robot_state->getGlobalLinkTransform(ee_link);
            visual_tools->publishAxisLabeled(start_pose, "Start");
            
            // 发布起点状?
            moveit_msgs::DisplayRobotState start_msg;
            moveit::core::robotStateToRobotStateMsg(*g_current_robot_state, start_msg.state);
            start_state_pub.publish(start_msg);

            g_current_robot_state->setJointGroupPositions(g_joint_model_group, goal->as<ob::RealVectorStateSpace::StateType>()->values);
            g_current_robot_state->updateLinkTransforms();
            const Eigen::Isometry3d goal_pose = g_current_robot_state->getGlobalLinkTransform(ee_link);
            visual_tools->publishAxisLabeled(goal_pose, "Goal");

            moveit_msgs::DisplayRobotState goal_msg;
            moveit::core::robotStateToRobotStateMsg(*g_current_robot_state, goal_msg.state);
            goal_state_pub.publish(goal_msg);

            visual_tools->trigger();
            ros::Duration(0.1).sleep();
            visual_tools->prompt("Press Next to continue");
        }

        og::SimpleSetup setup(si);// 设置规划器planner
        auto planner = std::make_shared<og::BITstar>(si); 
        setup.setPlanner(planner);
        setup.setStartAndGoalStates(start, goal);

        // 目标：最大化安全净空，路径长度不作为主要优化目标
        // 只保留净空惩罚目标，不使用路径长度目标（路径可以更长，但需要远离障碍物）
        // 若仍希望兼顾路径长度，可将 PathLengthOptimizationObjective 的权重设为很小的值（如 0.05）
        auto objective = std::make_shared<ob::MultiOptimizationObjective>(si);
        auto path_length_objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
        const double path_length_weight = std::max(0.0, 1.0 - planner_clearance_weight);
        objective->addObjective(std::make_shared<ClearancePenaltyObjective>(si, planner_clearance_margin,
                                                                             planner_clearance_epsilon),
                                planner_clearance_weight);     // 净空惩罚：主目标
        objective->addObjective(path_length_objective,
                                path_length_weight); // 路径长度：防止无限绕行
        // 为 BITstar 提供一个保守的 cost-to-go 启发式：只使用路径长度下界
        // 净空惩罚部分不提供额外乐观估计，保持启发式可采纳
        objective->setCostToGoHeuristic([path_length_objective, path_length_weight](const ob::State* state, const ob::Goal* goal) {
            return ob::Cost(path_length_weight * path_length_objective->costToGo(state, goal).value());
        });
        objective->lock();
        setup.setOptimizationObjective(objective);

        if (setup.solve(15.0) && setup.haveExactSolutionPath()) {
            auto& path = setup.getSolutionPath();// 获取规划器返回的路径

            // ================================================================
            // 路径优化：先备份原始路径，优化后若合法性被破坏则回退
            // 原因：simplifyMax / smoothBSpline 可能将路径状态推至障碍物边缘，
            //       导致后续 generateNearPathObstacle 中任何新障碍物都会命中路径
            // ================================================================
            og::PathGeometric backup_path(path); // 备份原始路径

            // 1. 先简化（去除冗余节点），此阶段有碰撞检测保护
            setup.getPathSimplifier()->simplifyMax(path);

            // 2. 先插值再平滑：插值使相邻节点足够近，BSpline 弓弦更短，不易穿越障碍物
            path.interpolate(std::max((unsigned int)(path.length() / 0.05), (unsigned int)50));
            setup.getPathSimplifier()->smoothBSpline(path, 3, 0.05);

            // 3. 优化后验证：检查路径是否仍然合法（无碰撞）
            if (!isPathSafe(path, si)) {
                ROS_WARN("Base %d: optimized path INVALID after smoothing. Reverting to original path.", base_idx);
                path = backup_path; // 回到原始路径
            } else {
                ROS_INFO("Base %d: optimized path valid, keeping.", base_idx);
            }

            // 4. 最终高密度插值（供后续障碍物距离检测使用）
            path.interpolate(std::max((unsigned int)(path.length() / 0.02), (unsigned int)100));
            
            if (debug) {
                visualizePath(path, visual_tools);
                visual_tools->prompt("Press Next to continue");
            }
            for (int variant_id = 0; variant_id < variants_per_path; ++variant_id)
            {
                // 对场景中的障碍物删除，重新添加基础障碍物（桌子 + 阶段0的全局障碍物）
                g_planning_scene->removeAllCollisionObjects();
                g_all_scene_obstacles.clear();
                for (size_t i = 0; i < base_obstacles.size(); ++i) {
                    addObstacleToScene("base_obs_" + std::to_string(i), base_obstacles[i]);
                    g_all_scene_obstacles.push_back(base_obstacles[i]);
                }

                // 每个 variant 开始时重置 RViz remote control 和 auto_continue 状态，
                // 防止上一个 variant 的 [Continue] 按钮状态延续到本次循环
                if (debug && preview_generated_obstacles && visual_tools) {
                    visual_tools->getRemoteControl()->setAutonomous(false);
                }

                ROS_INFO("Base Path %d, Scene Variant %d (Global Scene %d)", base_idx, variant_id, global_scene_id);

                bool preview_near_path_enabled = debug && preview_generated_obstacles && visual_tools;
                bool preview_extra_global_enabled = debug && preview_generated_obstacles && visual_tools;

                // ================================================================
                // 阶段2b：在末端执行器起止点连线上放置遮挡障碍物（路径已固定，不重新规划）
                // ================================================================
                int ee_blocked = 0;
                for (int i = 0; i < ee_blocking_obs_count * 10 && ee_blocked < ee_blocking_obs_count; ++i) {
                    Obstacle ob = generateEEBlockingObstacle(start.get(), goal.get());
                    std::string id = "ee_block_" + std::to_string(ee_blocked);
                    addObstacleToScene(id, ob);
                    // 验证：不能与已规划路径发生碰撞，且起止点仍有效
                    if (isStateValid(start.get()) && isStateValid(goal.get()) && isPathSafe(path, si)) {
                        g_all_scene_obstacles.push_back(ob);
                        ee_blocked++;
                    } else {
                        removeObstacleFromScene(id);
                    }
                }
                ROS_INFO("Variant %d: EE blocking obs placed: %d", variant_id, ee_blocked);
                
                // 阶段二：围绕路径生成极限障碍物
                // min_path_clearance: 路径连杆点与新障碍物表面的最小允许距离
                // 若障碍物距路径太近（< 此阈值），则 isPathSafe 几乎必然失败，提前过滤掉
                const double min_path_clearance = 0.035; // 单位 m，可根据机器人尺寸调整
                int planted = 0;
                std::vector<Obstacle> variant_obstacles;// 存放阶段二障碍物
                for (int i = 0; i < add_obs * 50 && planted < add_obs; ++i) {
                    Obstacle ob = generateNearPathObstacle(path);

                    // 预检：路径与候选障碍物的最小距离 >= min_path_clearance
                    // 若距离过小，则该障碍物直接与路径上的连杆碰撞，无需进入场景
                    double clearance = getObstacleDistToPath(path, ob);
                    if (clearance < min_path_clearance) {
                        continue; // 候选障碍物太靠近路径，直接丢弃
                    }

                    if (preview_near_path_enabled) {
                        std::string preview_id = "preview_near_path_" + std::to_string(i);
                        addObstacleToScene(preview_id, ob);// 场景中添加障碍物（预览用?
                        publishPlanningScene(scene_pub);// 更新场景?RViz
                        ros::Duration(0.1).sleep();
                        visual_tools->prompt("[Phase2] Near-path obstacle preview. [Next]=next candidate, [Continue]=skip remaining & finish planting");
                        removeObstacleFromScene(preview_id);
                        publishPlanningScene(scene_pub);
                        // 若用户按?[Continue]，后续不再个暂停，但仍继续补满本轮障碍物数量
                        if (visual_tools->getRemoteControl()->getAutonomous()) {
                            ROS_INFO("[Continue] pressed. Skipping remaining near-path obstacle previews for variant %d.", variant_id);
                            preview_near_path_enabled = false;
                        }
                    }
                    std::string id = "obs_p_" + std::to_string(planted);
                    addObstacleToScene(id, ob);
                    publishPlanningScene(scene_pub);
                    if (isPathSafe(path, si)) { 
                        variant_obstacles.push_back(ob);
                        g_all_scene_obstacles.push_back(ob);
                        planted++; 
                    }
                    else removeObstacleFromScene(id);
                }

                if (planted >= min_planted) { // 至少成功添加指定数量的瓶颈障碍物

                    // 阶段三：为了增加多样性，在全屢工作空间内再随机添加几个障碍?
                    int extra_global_obs = 0;
                    for (int i = 0; i < extra_global_obs_count*15 && extra_global_obs < extra_global_obs_count; ++i) {
                        Obstacle ob = generateGlobalWorkspaceObstacle(max_r*0.3, max_r*0.9, -0.05, 1.1);
                        if (preview_extra_global_enabled) {
                            // 每次进入 extra_global 阶段前重?continue 状（阶段二可能残留）
                            if (visual_tools->getRemoteControl()->getAutonomous()) {
                                visual_tools->getRemoteControl()->setAutonomous(false);
                            }
                            std::string preview_id = "preview_extra_global_" + std::to_string(i);
                            addObstacleToScene(preview_id, ob);
                            publishPlanningScene(scene_pub);
                            ros::Duration(0.1).sleep();
                            visual_tools->prompt("[Phase3] Extra global obstacle preview. [Next]=next candidate, [Continue]=skip remaining & finish");
                            removeObstacleFromScene(preview_id);
                            publishPlanningScene(scene_pub);
                            // 若用户按?[Continue]，后续不再个暂停，但仍继续补满本阶段障碍物数?
                            if (visual_tools->getRemoteControl()->getAutonomous()) {
                                ROS_INFO("[Continue] pressed. Skipping remaining extra global obstacle previews for variant %d.", variant_id);
                                preview_extra_global_enabled = false;
                            }
                        }
                        std::string id = "extra_global_" + std::to_string(i);
                        addObstacleToScene(id, ob);
                        publishPlanningScene(scene_pub);
                        if (isPathSafe(path, si)) { 
                            variant_obstacles.push_back(ob);
                            g_all_scene_obstacles.push_back(ob);
                            extra_global_obs++; 
                        }
                        else removeObstacleFromScene(id);
                    }

                    if (!isStateValid(start.get()) || !isStateValid(goal.get()) || !isPathSafe(path, si)) {
                        ROS_WARN("Variant %d: final validation failed before save. Skipping scene %d.", variant_id, global_scene_id);
                        continue;
                    }

                    ROS_INFO("%d-%d/%d: near path obs planted=%d, extra_global obs=%d, final validation passed", base_idx, variant_id, num_base_paths, planted, extra_global_obs);

                    // 保存环境数据 场景id 类型 x y z dim1 dim2 dim3
                    for (const auto& o : base_obstacles) {
                        env_file << global_scene_id << "," << o.type << "," << o.x << "," << o.y << "," << o.z << "," << o.dims[0] << "," << o.dims[1] << "," << o.dims[2] << "\n";
                    }
                    for (const auto& o : variant_obstacles) {
                        env_file << global_scene_id << "," << o.type << "," << o.x << "," << o.y << "," << o.z << "," << o.dims[0] << "," << o.dims[1] << "," << o.dims[2] << "\n";
                    }
                    // 保存路径数据 场景id 起止点类?0) 关节?,2,...（维度根据机器人自由度定?
                    for (size_t i = 0; i < path.getStateCount(); ++i) {
                        const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                        path_file << global_scene_id << ",0";
                        int var_count = g_joint_model_group->getVariableCount();
                        for (int j = 0; j < var_count; ++j) path_file << "," << s->values[j];
                        path_file << "\n";
                    }
                    env_file.flush(); path_file.flush();

                    if (debug) {              
                        publishPlanningScene(scene_pub);
                        
                        visual_tools->deleteAllMarkers();
                        visual_tools->publishText(text_pose, "Scene " + std::to_string(global_scene_id), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
                        
                        visual_tools->prompt("Press Next to continue");
                    }
                    global_scene_id++; 
                } else {
                    ROS_WARN("Variant %d: Only planted %d obstacles, less than min_planted=%d. Skipping save.", variant_id, planted, min_planted);
                }

            }
        } else {
            ROS_WARN("Failed to find base path for index %d", base_idx);
        }
    }
    env_file.close(); path_file.close();
    return 0;
}
