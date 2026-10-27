#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // 必须包含这个头文件
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cstdlib>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = moveit_visual_tools;

// 定义障碍物类型枚举
enum ShapeType {
    BOX = 0,
    SPHERE = 1,
    CYLINDER = 2
};

// 通用障碍物结构体
struct Obstacle {
    ShapeType type; 
    double x, y, z; 
    std::vector<double> dims; 

    Obstacle() : dims(3, 0.0) {} 
};

// 全局变量
planning_scene::PlanningScenePtr g_planning_scene;      
robot_state::RobotStatePtr g_current_robot_state;       
const robot_state::JointModelGroup* g_joint_model_group;

// ==========================================
// 自定义平滑算法：带有碰撞检测的切角平滑
// ==========================================
void customSmoothPath(og::PathGeometric& path, const ob::SpaceInformationPtr& si, int max_iterations = 3) {
    if (path.getStateCount() < 3) return;

    ob::StateSpacePtr space = si->getStateSpace();
    ob::State* temp1 = space->allocState();
    ob::State* temp2 = space->allocState();

    // 迭代多次，相当于打磨多遍
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool changed = false;
        
        // 创建一个新的路径容器，用于存放处理后的点
        og::PathGeometric new_path(si);
        new_path.append(path.getState(0)); // 起点不动

        // 遍历中间的每一个点 (跳过起点和终点)
        for (size_t i = 1; i < path.getStateCount() - 1; ++i) {
            const ob::State* prev = path.getState(i-1);
            const ob::State* curr = path.getState(i);
            const ob::State* next = path.getState(i+1);

            // 1. 计算两个切角点 (Cut Points)
            // 在 (prev -> curr) 的 70% 处取一点 A
            // 在 (curr -> next) 的 30% 处取一点 B
            // 这样就切掉了 curr 这个尖角
            space->interpolate(prev, curr, 0.7, temp1); // Point A
            space->interpolate(curr, next, 0.3, temp2); // Point B

            // 2. 关键：碰撞检测！
            // 检查直接从 A 走到 B 是否安全 (即切角操作是否会撞墙)
            if (si->checkMotion(temp1, temp2)) {
                // 安全！执行切角：加入 A 和 B，跳过 curr
                new_path.append(temp1);
                new_path.append(temp2);
                changed = true;
            } else {
                // 不安全（说明切太多了，或者障碍物就在角上）
                // 保守策略：尝试切小一点 (比如 90% 和 10%)
                space->interpolate(prev, curr, 0.9, temp1);
                space->interpolate(curr, next, 0.1, temp2);
                
                if (si->checkMotion(temp1, temp2)) {
                    new_path.append(temp1);
                    new_path.append(temp2);
                    changed = true;
                } else {
                    // 实在切不动，保留原点 (这是最坏情况，保证无碰撞)
                    new_path.append(curr);
                }
            }
        }
        
        new_path.append(path.getState(path.getStateCount() - 1)); // 终点不动
        
        // 更新路径
        path = new_path;
        
        // 如果这一轮没动任何点，说明已经很平滑或者卡死了，提前退出
        if (!changed) break;
    }

    space->freeState(temp1);
    space->freeState(temp2);
}
// ==========================================
// 最终安全检查：遍历路径上的每一个点，确保无碰撞
// ==========================================
bool isPathSafe(const og::PathGeometric& path, const ob::SpaceInformationPtr& si) {
    // 1. 检查每一个离散点状态是否有效
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const ob::State* state = path.getState(i);
        if (!si->isValid(state)) {
            // 发现碰撞点！
            return false; 
        }
    }

    // 2. (可选) 更加严格的检查：检查点与点之间的连线
    // 如果点非常密集 (5mm)，上面的检查通常够了。
    // 如果为了绝对安全，可以检查每两个相邻点之间的 Motion
    for (std::size_t i = 0; i < path.getStateCount() - 1; ++i) {
        const ob::State* s1 = path.getState(i);
        const ob::State* s2 = path.getState(i+1);
        if (!si->checkMotion(s1, s2)) {
            return false;
        }
    }

    return true;
}
// 计算机械臂的最大理论臂展
double computeMaxReach(const robot_model::JointModelGroup* group) {
    double max_reach = 0.0;
    const std::vector<const robot_model::LinkModel*>& links = group->getLinkModels();
    for (const auto* link : links) {
        double link_length = link->getJointOriginTransform().translation().norm();       
        max_reach += link_length;
    }
    return max_reach;
}

// 碰撞检测器
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

// 将障碍物添加到 MoveIt 场景
void addObstacle2Sence(std::string frame, int obstacle_id, const Obstacle& obstacleInfo)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame;
    collision_object.id = "box_" + std::to_string(obstacle_id);

    shape_msgs::SolidPrimitive primitive;
    if (obstacleInfo.type == BOX) {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = obstacleInfo.dims[0];
        primitive.dimensions[primitive.BOX_Y] = obstacleInfo.dims[1];
        primitive.dimensions[primitive.BOX_Z] = obstacleInfo.dims[2];
    }
    else if (obstacleInfo.type == SPHERE) {
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[primitive.SPHERE_RADIUS] = obstacleInfo.dims[0];
    }
    else if (obstacleInfo.type == CYLINDER) {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = obstacleInfo.dims[0]; 
        primitive.dimensions[primitive.CYLINDER_RADIUS] = obstacleInfo.dims[1]; 
    }
    
    geometry_msgs::Pose box_pose;
    box_pose.position.x = obstacleInfo.x;
    box_pose.position.y = obstacleInfo.y;
    box_pose.position.z = obstacleInfo.z;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    g_planning_scene->processCollisionObjectMsg(collision_object);
}

// 生成随机场景并保存 CSV 调用addObstacle2Sence 将障碍物添加到场景中
void generateRandomObstacles(int num_obstacles, std::ofstream& env_file, int scene_id) {
    g_planning_scene->removeAllCollisionObjects();
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static double max_radius = 0.0;
    if (max_radius == 0.0) {
        max_radius = computeMaxReach(g_joint_model_group);// 计算机械臂的最大臂展
        if (max_radius < 0.5) max_radius = 0.85; 
        ROS_INFO("Max Reach Calculated: %.2f m", max_radius);
    }
    double min_radius = 0.30 - 0.1; 

    // 设置随机采样的范围
    std::uniform_real_distribution<> radius_dist(min_radius, max_radius*0.95);// 位置半径范围
    std::uniform_real_distribution<> angle_dist(-M_PI, M_PI);// 角度范围
    std::uniform_real_distribution<> z_dist(0.0, 0.6);// 高度范围
    std::uniform_real_distribution<> size_dist(0.05, 0.30);// 尺寸大小范围 
    std::uniform_real_distribution<> radius_size_dist(0.05, 0.15);
    std::uniform_int_distribution<> type_dist(0, 2);  
    
    // 默认添加一个桌子 让机械臂不会向下规划
    Obstacle table;
    table.type = CYLINDER;
    table.x = 0;
    table.y = 0;
    table.z = -0.0101;
    table.dims[0] = 0.02;
    table.dims[1] = 1;
    addObstacle2Sence("base_link", 0, table);

    env_file << scene_id << "," << table.type << "," 
                << table.x << "," << table.y << "," << table.z << "," 
                << table.dims[0] << "," << table.dims[1] << "," << table.dims[2] << "\n";  

    for (int i = 1; i <= num_obstacles; ++i) {
        Obstacle obs;
        double r = radius_dist(gen);
        double theta = angle_dist(gen);
        obs.x = r * std::cos(theta);
        obs.y = r * std::sin(theta);
        obs.z = z_dist(gen);

        int type_val = type_dist(gen);
        obs.type = static_cast<ShapeType>(type_val);

        if (obs.type == BOX) {
            obs.dims[0] = size_dist(gen); 
            obs.dims[1] = size_dist(gen); 
            obs.dims[2] = size_dist(gen); 
        } 
        else if (obs.type == SPHERE) {
            obs.dims[0] = radius_size_dist(gen);
        }
        else if (obs.type == CYLINDER) {
            obs.dims[0] = size_dist(gen);        
            obs.dims[1] = radius_size_dist(gen); 
        }
        else {
            continue;
        }

        addObstacle2Sence("base_link", i, obs);

        env_file << scene_id << "," << obs.type << "," 
                 << obs.x << "," << obs.y << "," << obs.z << "," 
                 << obs.dims[0] << "," << obs.dims[1] << "," << obs.dims[2] << "\n";      
    }
}

void getRobotConf(ob::RealVectorBounds& bounds, int dim)
{
    const std::vector<std::string>& vars = g_joint_model_group->getVariableNames();
    const robot_model::RobotModel& robot_model = g_joint_model_group->getParentModel();

    for (unsigned int i = 0; i < dim; ++i)
    {
        const std::string& var_name = vars[i];
        const robot_model::VariableBounds& b = robot_model.getVariableBounds(var_name);

        if (b.position_bounded_) {
            bounds.setLow(i, b.min_position_);
            bounds.setHigh(i, b.max_position_);
        } else {
            bounds.setLow(i, -M_PI);
            bounds.setHigh(i, M_PI);
        }
    }
}

// 修正后的可视化函数：手动计算 FK 并画线
void visualizePath(og::PathGeometric& path, 
                   moveit_visual_tools::MoveItVisualToolsPtr& visual_tools,
                   const robot_state::JointModelGroup* group) 
{
    if (!visual_tools) return;

    // 1. 准备一个容器存这类点
    std::vector<geometry_msgs::Point> path_points;
    
    // 2. 获取该组的末端连杆名称 (End Effector Link)
    // getLinkModelNames().back() 通常能拿到最后一个连杆，比如 wrist3_Link
    const std::vector<std::string>& link_names = group->getLinkModelNames();
    if (link_names.empty()) return;
    const std::string& ee_link_name = link_names.back();

    // 3. 遍历 OMPL 路径的所有状态
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        
        // 更新机器人的姿态
        g_current_robot_state->setJointGroupPositions(group, state->values);
        
        // 【关键】必须显式更新连杆变换，否则 getGlobalLinkTransform 拿到的还是旧数据
        g_current_robot_state->updateLinkTransforms(); 
        
        // 计算正运动学 (FK)，拿到末端的位置
        const Eigen::Isometry3d& pose = g_current_robot_state->getGlobalLinkTransform(ee_link_name);
        
        // 存入 vector
        geometry_msgs::Point p;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        path_points.push_back(p);
    }

    // 4. 直接调用 publishPath (这个函数比 publishTrajectoryLine 稳定得多)
    // LIME_GREEN 是亮绿色，SMALL 是线条粗细
    visual_tools->publishPath(path_points, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    
    // 5. 立即发送
    visual_tools->trigger();
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "ompl_data_generator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 用于读取私有参数

    // 读取 debug 参数
    bool debug_mode = false;
    private_nh.param("debug", debug_mode, false);
    ROS_INFO_STREAM("Debug Mode: " << (debug_mode ? "ENABLED (Visualizing)" : "DISABLED (Fast Generation)"));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 MoveIt 模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    if (!kinematic_model) {
        ROS_ERROR("Failed to load robot model.");
        return -1;
    }

    g_planning_scene.reset(new planning_scene::PlanningScene(kinematic_model));
    g_current_robot_state.reset(new robot_state::RobotState(kinematic_model));
    
    std::string group_name = "arm"; 
    g_joint_model_group = g_current_robot_state->getJointModelGroup(group_name);
    
    if (!g_joint_model_group) {
        ROS_ERROR("Joint group '%s' not found!", group_name.c_str());
        return -1;
    }

    // 初始化可视化工具 (仅在 Debug 模式下)
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
    ros::Publisher scene_pub;
    ros::Publisher start_state_pub;
    ros::Publisher goal_state_pub;

    if (debug_mode) {
        // 使用智能指针初始化
        visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(kinematic_model->getModelFrame(), "/rviz_visual_tools", kinematic_model));
        visual_tools->loadRemoteControl();
        visual_tools->deleteAllMarkers();
        
        // 初始化场景发布者
        scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
        start_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_start_state", 10, true);
        goal_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_goal_state", 10, true);

    }

    // 初始化 OMPL
    unsigned int dim = g_joint_model_group->getVariableCount(); // 获取维度
    auto space(std::make_shared<ob::RealVectorStateSpace>(dim));    // 状态空间
    ob::RealVectorBounds bounds(dim);// 设置状态空间的范围
    getRobotConf(bounds, dim);  // 设置关节限位
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(isStateValid);
    si->setup();

    const char* home_dir = std::getenv("HOME");// 设置保存路径
    std::string path_prefix = std::string(home_dir) + "/aubo/arm_ws/src/motion_planning/data/";
    std::ofstream env_file(path_prefix + "training_envs.csv");
    std::ofstream path_file(path_prefix + "training_paths.csv");

    if (!env_file.is_open() || !path_file.is_open()) return -1;

    int total_scenes = 100;
    int paths_per_scene = 20;

    // 为了显示文字的位置 (机器人上方)
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    for (int scene_id = 0; scene_id < total_scenes; ++scene_id) {
        if (!ros::ok()) break; 

        ROS_INFO("Generating Scene %d / %d", scene_id, total_scenes);
        generateRandomObstacles(15, env_file, scene_id);        // 障碍物个数

        // 【Debug】发布障碍物和文字
        if (debug_mode) {
            moveit_msgs::PlanningScene scene_msg;
            g_planning_scene->getPlanningSceneMsg(scene_msg);

            scene_msg.is_diff = true;
            scene_pub.publish(scene_msg);

            ros::spinOnce(); 
            ros::Duration(0.5).sleep(); // 暂停0.5秒，给RViz一点反应时间
            ROS_INFO("Scene published to RViz.");

            visual_tools->deleteAllMarkers(); 
            // 这里的颜色和尺寸必须用 rviz_visual_tools 命名空间
            visual_tools->publishText(text_pose, "Scene " + std::to_string(scene_id), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            visual_tools->trigger();
            ros::Duration(0.1).sleep(); // 给 RViz 一点接收时间
        }

        for (int path_id = 0; path_id < paths_per_scene; ++path_id) {
            if (!ros::ok()) break;

            if (debug_mode) {
                visual_tools->deleteAllMarkers(); // 放入删除指令
                visual_tools->publishText(text_pose, "Scene " + std::to_string(scene_id), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
                visual_tools->trigger();          // 【重要】立即执行删除！
            }

            og::SimpleSetup setup(si);
            // auto planner(std::make_shared<og::RRTstar>(si));// 设置算法
            // auto planner(std::make_shared<og::InformedRRTstar>(si));// RRT* Informed
            auto planner(std::make_shared<og::BITstar>(si));// Batch Informed Trees BIT*
            // 开启修剪 (Pruning)，这能帮助算法在找到初始解后快速丢弃无用的样本，加速收敛到最优解
            planner->setPruning(true);
            // 增加每批次的采样数，从默认 100 增加到 200，适合复杂场景
            planner->setSamplesPerBatch(200);
            // planner->setRange(0.1);// 设置步长
            setup.setPlanner(planner);
            
            ob::ScopedState<> start(space);
            ob::ScopedState<> goal(space);
            bool valid_endpoints = false;
            bool is_hard_sample = false;

            int attempts = 0;
            // 判断创建的起始、结束位置的状态是否有效
            while(!valid_endpoints && ros::ok() && attempts < 1000) {
                attempts++;
                start.random();
                goal.random();
                if (isStateValid(start.get()) && isStateValid(goal.get())) {
                    // 检查直线路径是否有碰撞 (简单的可视性检查)
                    // checkMotion 检查两点间直线运动是否有效。
                    // 如果 checkMotion 返回 false，说明直线有碰撞 -> 困难样本
                    if (!si->checkMotion(start.get(), goal.get())) { 
                        is_hard_sample = true; 
                    }
                    // 策略：如果我们需要更多困难样本，且当前是简单样本，则有一定概率丢弃它
                    if (!is_hard_sample) {
                        // 80% 的概率丢弃简单样本，只保留 20%
                        if ((rand() % 100) < 80) continue; 
                    }
                    valid_endpoints = true;
                }
            }
            if (attempts >= 1000) {
                ROS_WARN("Failed to sample valid endpoints, skipping...");
                continue;
            }
            if (!ros::ok()) break;

            auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);// 添加优化目标
            double dist = space->distance(start.get(), goal.get());// 计算直线距离
            double threshold = dist * 1.15;          // 优化阈值
            objective->setCostThreshold(ob::Cost(threshold));// 设置阈值
            setup.setOptimizationObjective(objective);
            
            setup.setStartAndGoalStates(start, goal);// 设置 规划目标（起始、结束）

            // 【Debug】显示起点和终点 (需要计算正运动学)
            if (debug_mode) {
                // 计算 Start 的位姿
                const double* start_val = start->as<ob::RealVectorStateSpace::StateType>()->values;
                g_current_robot_state->setJointGroupPositions(g_joint_model_group, start_val);
                g_current_robot_state->updateLinkTransforms();

                moveit_msgs::DisplayRobotState start_msg;
                moveit::core::robotStateToRobotStateMsg(*g_current_robot_state, start_msg.state);// 数据类型转化
                start_state_pub.publish(start_msg);

                // 获取末端执行器名称 (通常是 link list 的最后一个)
                const std::string& ee_link = g_joint_model_group->getLinkModelNames().back();
                const Eigen::Isometry3d start_pose = g_current_robot_state->getGlobalLinkTransform(ee_link);

                // 计算 Goal 的位姿
                const double* goal_val = goal->as<ob::RealVectorStateSpace::StateType>()->values;
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

            ROS_INFO("  Planning path %d (RRT*)...", path_id);
            ob::PlannerStatus solved = setup.solve(25.0);

            // 判断是否找到解决方案
            if (solved && setup.haveExactSolutionPath()) {
                og::PathGeometric& path = setup.getSolutionPath();	
                og::PathSimplifierPtr ps = setup.getPathSimplifier();	//先进行几何简化 (这一步会去除多余的拐点)
                ps->simplifyMax(path);       // shortcut 和 reduceVertices
                customSmoothPath(path, si, 5);// 代替原来的 smoothBSpline。迭代 5 次效果通常很好。
                
                ps->smoothBSpline(path);
                //setup.simplifySolution();	
                
                // 3. 插值 (解决数据稀疏)
                // 警告：OMPL的 interpolate 是线性插值。
                // 为了让数据在电机上更平滑，我们需要足够密集的点，然后交给 Python 做三次样条拟合。
                // 建议：保持插值，但增加密度。
                double path_len = path.length();
                // 将步长从 0.05 减小到 0.02，生成更密集的点，捕捉 B样条的曲率
                unsigned int num_points = static_cast<unsigned int>(path_len / 0.05); 
                num_points = std::max(num_points, (unsigned int)100); // 至少 100 个点
                path.interpolate(num_points);
                
                // =================================================
                // 4. 【新增】最终全路径碰撞检测 (Final Safety Check)
                // =================================================
                if (!isPathSafe(path, si)) {
                    ROS_WARN("Path %d failed final safety check (collision detected after smoothing). Discarding...", path_id);
                    path_id--; // 关键：回退 path_id，让程序重新生成这一条，保证总数不变
                    continue;  // 跳过保存，进入下一次循环
                }

                // 【Debug】显示路径并暂停
                if (debug_mode) {
                    ROS_INFO("Visualizing path...");
                    visualizePath(path, visual_tools, g_joint_model_group);
                    
                    // 暂停等待 Next 按钮
                    visual_tools->prompt("Press 'Next' in RViz to continue"); 
                }

                // 保存数据
                for (std::size_t i = 0; i < path.getStateCount(); ++i) {
                    const auto *joint_state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                    path_file << scene_id << "," << path_id;
                    for (int j = 0; j < 6; ++j) {
                        path_file << "," << joint_state->values[j];
                    }
                    path_file << "\n";
                }
                ROS_INFO("  Saved %ld dense points.", path.getStateCount());

            } else {
                ROS_WARN("  Planning failed.");
            }
            setup.clear();
        }
    }

    env_file.close();
    path_file.close();
    ROS_INFO("Data generation complete!");

    // 显式释放全局智能指针，防止在 ROS 节点关闭后析构导致的 Crash
    g_planning_scene.reset();
    g_current_robot_state.reset();

    return 0;
}
