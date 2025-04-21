#include <rclcpp/rclcpp.hpp>                                          // ROS2 C++客户端库
#include <moveit/planning_scene/planning_scene.h>                     // Moveit规划场景
#include <moveit/planning_scene_interface/planning_scene_interface.h> // 规划场景接口
#include <moveit/task_constructor/task.h>                             // MTC 任务规划器
#include <moveit/task_constructor/solvers.h>                          // MTC求解器
#include <moveit/task_constructor/stages.h>                           // MTC任务阶段
#include <thread>                                                     // std::this_thread::sleep_for
#include <chrono>                                                     // 时间库
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // TF2几何消息（新版本）
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // TF2几何消息（旧版本）
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp> // TF2几何消息（新版本）
#else
#include <tf2_eigen/tf2_eigen.h> // TF2几何消息（旧版本）
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo"); // 日志定义
namespace mtc = moveit::task_constructor;                                   // MTC 命名空间别名

const std::string BASE_LINK = "base_link";                                                   // 基础坐标系
const std::string TABLE_ID = "table";                                                        // 桌子物体ID
const std::string TARGET_ID = "object";                                                      // 目标物体ID
const std::vector<std::string> GRIPPER_JOINT_NAMES = {"gripper_1_joint", "gripper_2_joint"}; // 夹抓关节名
const std::vector<double> GRIPPER_OPEN = {0.65, 0.65};                                       // 夹爪打开位置
const std::vector<double> GRIPPER_GRASP = {0.1, 0.1};                                        // 夹爪抓取位置

class PickPlaceDemo
{
public:
    PickPlaceDemo(const rclcpp::NodeOptions &options); // 构造函数

    // 获取节点基础接口
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask(); // 执行任务

    void setupPlanningScene(); // 设置规划场景

private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();        // 创建MTC任务
    mtc::Task task_;               // MTC任务对象
    rclcpp::Node::SharedPtr node_; // ROS2节点
};

/// @brief 获取节点接口
/// @return
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PickPlaceDemo::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

PickPlaceDemo::PickPlaceDemo(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("moveit_pick_place_demo", options)}
{

}

// 执行任务
void PickPlaceDemo::doTask()
{
    task_ = createTask(); // 创建任务

    try
    {
        task_.init(); // 初始化
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e); // 错误处理
        return;
    }

    // 规划任务最多5个解
    if (!task_.plan(5 /* max_solutions */))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    // 发布可视化解决方案
    task_.introspection().publishSolution(*task_.solutions().front());

    // 执行任务
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

mtc::Task PickPlaceDemo::createTask()
{
    // STEP 1： 任务初始化
    mtc::Task task;                      // 创建MTC任务对象
    task.stages()->setName("demo task"); // 设置任务名称
    task.loadRobotModel(node_);          // 加载机器人模型

    // STEP 2： 关键参数配置
    // 定义规划组名称
    const auto &arm_group_name = "xarm";
    const auto &hand_group_name = "gripper";
    const auto &hand_frame = "gripper_centor_link"; // 逆运动学的参考坐标系
    // 设置任务属性
    task.setProperty("group", arm_group_name); // 机械臂规划组
    task.setProperty("eef", "hand");           // 末端执行器
    task.setProperty("ik_frame", hand_frame);  // IK参考坐标系

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    // mtc::Stage 可能是 MoveIt Task Constructor（MTC）框架中的一个类，表示任务的一个阶段（stage）
    // 表明这个指针的用途是将当前的 state（可能是机器人的状态或任务的状态）传递给抓取姿势生成器（grasp pose generator）。
    // 具体来说，这个指针的作用是将当前的 state 传递给 grasp pose generator，以便它可以在生成抓取姿势时使用当前的机器人状态。
    mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    // 创建规划器
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);           // 采样规划器
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>(); // 插值规划器
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                 // 笛卡尔路径规划器配置
    cartesian_planner->setMaxVelocityScalingFactor(1.0);                                      // 最大速度比例
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);                                  // 最大加速度比例
    cartesian_planner->setStepSize(.01);                                                      // 长度步长

    // STEP 3： 核心阶段，由多个阶段(Stage)组成
    // （1）初始阶段
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get(); // 获取指针，.get() 是智能指针的成员函数，用于获取它所管理的原始指针，不转移所有权
    task.add(std::move(stage_state_current));      // std::move，将智能指针的所有权转移给 task.add()，转移所有权

    // （2）打开夹爪阶段
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("Open_gripper", interpolation_planner); // 创建 moveto 阶段
    stage_open_hand->setGroup(hand_group_name);                                       // "gripper"
    stage_open_hand->setGoal("Open_gripper");                                         // group_state name
    task.add(std::move(stage_open_hand));                                             // 转移后，stage_open_hand变为nullptr，task管理生命期，结束后stage_open_hand会自动析构

    // clang-format off
    // （3）移动到抓取位置阶段
    // connect 阶段用于连接两个状态（当前状态和目标状态），生成可行的运动路径
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", // 阶段的名称，用于调试和日志记录
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } }); // 使用采样规划器（OMPL）规划机械臂到抓取点的路径
    // clang-format on
    stage_move_to_pick->setTimeout(5.0);                                    // 设置超时时间间隔
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT); // 从父阶段继承属性
    task.add(std::move(stage_move_to_pick));                                // 添加阶段

    mtc::Stage *attach_object_stage = nullptr; // Forward attach_object_stage to place pose generator

    // This is an example of SerialContainer usage. It's not strictly needed here.
    // In fact, `task` itself is a SerialContainer by default.
    // （4）抓取序列（核心）
    {
        // 通过SerialContainer可以将多个阶段组合在一起，形成一个序列，每个阶段都会按照顺序执行
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");            // 创建串行容器
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"}); // 将任务task中的属性(eef, group,ik_frame)暴露给grasp容器的属性，以便子阶段可以访问这些属性
        // clang-format off
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" }); // 属性初始化配置，从父阶段继承属性，包括eef, group, ik_frame属性，以便子阶段（grasp序列任务）可以访问这些属性
        // clang-format on
        /****************************************************
                         （4.1）接近物体
         ***************************************************/
        {
            // clang-format off
            // approach：阶段名称，cartesian_planner：指定使用笛卡尔空间规划器（确保直线运动）
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner); // MoveRelateive：相对运动阶段，用于移动机器人的末端执行器（eef）
            // clang-format on
            // 设置阶段属性
            stage->properties().set("marker_ns", "approach_object");              // marker_ns：命名空间，用于在Rviz中可视化。设置 RViz 中可视化标记的命名空间（approach_object）
            stage->properties().set("link", hand_frame);                          // link：指定移动的参考坐标系，这里设置为hand_frame（夹爪的参考坐标系）
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"}); // 从父阶段继承属性，包括group属性
            stage->setMinMaxDistance(0.02, 0.2);                                   // 设置运动范围，最小距离为0.1，最大距离为0.2

            // Set hand forward direction
            // 设置运动方向
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame; // 方向基于夹爪坐标系
            vec.vector.x = 1.0;               // 沿末端坐标系x轴接近
            stage->setDirection(vec);         // 设置运动方向
            grasp->insert(std::move(stage));  // 将阶段添加到抓取序列中
        }

        /****************************************************
                       Generate Grasp Pose
                       （4.2）生成抓取姿态
         ***************************************************/
        {
            // Sample grasp pose
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose"); // 生成物体抓取姿态（位姿）的候选集合
            stage->properties().configureInitFrom(mtc::Stage::PARENT);                            // 配置抓取姿态生成器的属性，包括eef、group等
            stage->properties().set("marker_ns", "grasp_pose");                                   // 设置RViz中可视化标记的命名空间为 grasp_pose,用于调试抓取姿态
            stage->setPreGraspPose("Open_gripper");                                               // 指定夹爪在抓取前的预抓取姿态（"Open_gripper" 是预定义的夹爪张开状态）。
            stage->setObject("object");                                                           // 设置目标物体的 ID（与规划场景中添加的物体 ID TARGET_ID 一致）。
            stage->setAngleDelta(M_PI / 12);                                                      // 设置绕物体 Z 轴旋转的步长（这里为 15°），用于生成多个抓取姿态候选。
            stage->setMonitoredStage(current_state_ptr);                                          // 关联到当前状态阶段（current_state_ptr），确保抓取姿态基于最新的机器人状态计算。

            // This is the transform from the object frame to the end-effector frame
            // 定义夹爪坐标系相对于物体坐标系的位姿变换。
            Eigen::Isometry3d grasp_frame_transform; // 设置夹爪相对于物体的位置和朝向
            Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(M_PI / 20, Eigen::Vector3d::UnitZ()); // 通过四元数q设置夹爪的朝向
            grasp_frame_transform = Eigen::Translation3d(0, 0, 0.03) * q;                  // 生成变换矩阵，描述夹爪相对于物体的位置和朝向
            // stage->setGraspPose(grasp_frame_transform);                                    // 设置抓取姿态的位姿变换。
            // stage->setMonitoredStage(attach_object_stage);                                 // 关联到附加物体阶段（attach_object_stage），确保抓取姿态生成器在附加物体时能够正确更新。

            // clang-format off
            // 计算逆运动学
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));// 将生成的抓取姿态转换为机械臂的关节空间解（IK）
            // clang-format on
            wrapper->setMaxIKSolutions(8);                                                   // 设置最大IK解数量（尝试最多8个解）
            wrapper->setMinSolutionDistance(1.0);                                            // 设置解之间的最小关节空间距离（避免相似的冗余解）
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);                          // 设置I坐标系，grasp_frame_transform 夹爪相对于物体的位姿，hand_frame 夹爪坐标系
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});   // 从父阶段继承属性，包括eef和group属性
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"}); // 从接口继承属性，包括target_pose属性
            grasp->insert(std::move(wrapper));                                               // 将阶段添加到抓取序列
        }

        {
            // clang-format off
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)"); // 动态修改规划场景，允许夹爪与物体发生碰撞
            // 在抓取任务中，夹爪需要靠近或接触物体，此时需临时允许两者碰撞，避免规划器误判为冲突。
            // 设置碰撞规则
            // 底层机制：allowCollisions会更新AllowedCollisionMatrix(ACM),告知规划器忽略指定对象的碰撞检测
            stage->allowCollisions("object", // 目标物体的ID（需与规划场景中的碰撞物体ID一致）
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
            // clang-format on
            grasp->insert(std::move(stage)); // 插入到抓取容器
        }

        /****************************************************
         *                （4.3）执行闭合状态
         ***************************************************/
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("Close_gripper");
            grasp->insert(std::move(stage));
        }
        /****************************************************
         *                （4.4）附加物体
         *   将目标物体object动态附着在机械臂的末端执行器，
         *   从而实现抓取操作中的物体与机器人的联动运动
         ***************************************************/
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object"); // 动态修改规划场景
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }
        /****************************************************
         ***************************************************/
        {
            // clang-format off
            // 传播器阶段，相对运动阶段，用于移动机器人的末端执行器（eef）
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner); // 创建MoveRelative对象，用于规划末端执行器的相对运动路径。
            // clang-format on
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"}); // 从父阶段（SerialContainer）继承属性，包括group属性
            stage->setMinMaxDistance(0.1, 0.3);                                   // 最小/最大运动距离（单位：米）
            stage->setIKFrame(hand_frame);                                        // 设置逆运动学参考坐标系（如夹爪工具帧）
            stage->properties().set("marker_ns", "lift_object");                  // 设置可视化标记的命名空间

            // Set upward direction
            // 运动方向定义
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "base_link"; // 参考坐标系为基座
            vec.vector.z = 1.0;                // 沿基座坐标系的Z轴正方向运动（垂直向上）
            stage->setDirection(vec);          // 设置运动方向
            grasp->insert(std::move(stage));   // 将运动阶段添加到抓取序列
        }
        task.add(std::move(grasp)); // 插入到抓取容器
    }

    // connect 阶段
    // 连接两个组的运动，确保它们在同一时间点上达到目标状态
    {
        // clang-format off
        // Connect 阶段，用于连接两个组的运动，确保它们在同一时间点上达到目标状态
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place", // 阶段名称
            // 指定不同规划组使用的规划器
            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }, // 机械臂使用采样规划器
                                                    { hand_group_name, interpolation_planner } }); // 夹爪使用插值规划器
        // clang-format on
        stage_move_to_place->setTimeout(5.0);                                    // 设置规划超时时间（5s）
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT); // 继承父阶段属性
        task.add(std::move(stage_move_to_place));                                // 添加到任务中
    }

    // （5）放置任务
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");           // 创建一个名为 place object的 SerialContainer容器，用于按顺序执行放置子阶段
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"}); // 将place容器的属性暴露给子阶段
        // clang-format off
        place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            { "eef", "group", "ik_frame" }); // 从父阶段继承属性
        // clang-format on

        /****************************************************
      ---- *               Generate Place Pose                *
                            生成放置位姿
         ***************************************************/
        {
            // Sample place pose
            // 生成放置位姿
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose"); // 创建一个名为 generate place pose的 GeneratePlacePose子阶段
            stage->properties().configureInitFrom(mtc::Stage::PARENT);                            // 从父阶段继承属性
            stage->properties().set("marker_ns", "place_pose");                                   // 设置可视化标记的命名空间
            stage->setObject("object");                                                           // 设置目标物体的ID（需与规划场景中的物体ID一致）

            // 设置目标位姿
            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = "base_link"; // 参考坐标系为机械臂基座
            target_pose_msg.pose.position.x = 0.32;        // x轴位置
            target_pose_msg.pose.position.y = -0.32;
            target_pose_msg.pose.position.z = 0.22 / 2.0;
            tf2::Quaternion orientation; // 位姿
            orientation.setRPY(0, 0, -M_PI / 4);
            target_pose_msg.pose.orientation = tf2::toMsg(orientation);
            stage->setPose(target_pose_msg); // 设置目标位姿

            // 关联物体附着阶段
            stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

            // Compute IK 计算逆运动学解（ComputeIK）
            // clang-format off
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage)); // 将笛卡尔位姿转换为关节状态
            // clang-format on
            wrapper->setMaxIKSolutions(2);                                                   // 最多生成2个IK解
            wrapper->setMinSolutionDistance(1.0);                                            // 解之间的最小距离
            wrapper->setIKFrame("object");                                                   // 以物体坐标系为参考计算IK
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});   // 继承属性
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"}); // 继承接口属性
            place->insert(std::move(wrapper));                                               // 将子阶段添加到place容器中
        }

        /****************************************************
      ---- *               机械臂末端的打开动作                 *
         ***************************************************/
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner); // 创建一个名为 open hand的 MoveTo子阶段
            stage->setGroup(hand_group_name);                                                       // 夹爪规划组
            stage->setGoal("Open_gripper");                                                         // 夹爪目标状态
            place->insert(std::move(stage));                                                        // 将子阶段添加到place容器中
        }

        /****************************************************
      ---- *               禁止夹爪与目标物体之间的碰撞检测                 *
         ***************************************************/
        {
            // clang-format off
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)"); // 专门用于动态修改规划场景的阶段，支持碰撞规则调整、物体附着/分离等操作
            stage->allowCollisions("object", // 碰撞物ID
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    false); // false禁止碰撞
            // clang-format on
            place->insert(std::move(stage)); // 将阶段添加到SerialContainer中
        }

        /****************************************************
      ---- *       解除物体与机械臂末端执行器（如夹爪）的附着关系                 *
         ***************************************************/
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object"); // 专门用于动态修改规划场景
            stage->detachObject("object", hand_frame);                                        // 解除物体与机械臂末端执行器的附着关系
            place->insert(std::move(stage));                                                  // 将阶段添加到SerialContainer中
        }

        /****************************************************
      ---- *                   安全撤离动作                 *
         ***************************************************/
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner); // 创建名为 retreat的相对运动阶段，使用笛卡尔规划器确保直线运动
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});                   // 从父阶段继承属性
            stage->setMinMaxDistance(0.1, 0.15);                                                    // 设置最小/最大运动距离
            stage->setIKFrame(hand_frame);                                                          // 设置逆运动学参考坐标系（如夹爪工具帧）
            stage->properties().set("marker_ns", "retreat");                                        // 设置可视化标记的命名空间

            // Set retreat direction
            geometry_msgs::msg::Vector3Stamped vec; // 运动方向定义
            vec.header.frame_id = "base_link";      // 参考坐标系为基座
            vec.vector.z = 1.0;                     // 沿基座坐标系的Z轴正方向运动（垂直向上）
            stage->setDirection(vec);               // 设置运动方向
            place->insert(std::move(stage));        // 将运动阶段添加到抓取序列
        }
        task.add(std::move(place));
    }

    /****************************************************
    ---- *           返回到预定义的就位状态                 *
    ***************************************************/
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner); // 创建名为 return home的 MoveTo子阶段
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});                     // 从父阶段继承属性
        stage->setGoal("ready");                                                                  // 设置目标状态为 ready
        task.add(std::move(stage));
    }
    return task;
}

/// @brief 接口动态设置规划场景
void PickPlaceDemo::setupPlanningScene()
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(2);

    collision_objects[0].id = TABLE_ID;               // 桌面
    collision_objects[0].header.frame_id = BASE_LINK; // 参考坐标系（如机械臂基座）

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX; // 长方体
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions = {0.5, 0.5, 0.01}; // 长宽高（单位：米）

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = -0.01;
    collision_objects[0].operation = collision_objects[0].ADD; // 操作为添加物体

    // -----------------------------------------------------

    collision_objects[1].id = TARGET_ID;              // 目标物体
    collision_objects[1].header.frame_id = BASE_LINK; // 参考坐标系（如机械臂基座）

    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX; // 小方块
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions = {0.05, 0.05, 0.22}; // 长宽高（单位：米）
    collision_objects[1].pose.position.x = 0.47;
    collision_objects[1].pose.position.y = 0.0;
    collision_objects[1].pose.position.z = 0.11;
    collision_objects[1].pose.orientation.w = 1; // 无旋转
    collision_objects[1].operation = collision_objects[1].ADD;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // 场景更新与同步
    planning_scene_interface.addCollisionObjects(collision_objects);             // 添加物体
    planning_scene_interface.applyCollisionObjects(collision_objects);           // 应用物体
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<PickPlaceDemo>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                     {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });
    
    RCLCPP_INFO_STREAM(LOGGER, "Pick and Place demo is ready.");

    // 添加障碍物和目标物体
    mtc_task_node->setupPlanningScene();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}
