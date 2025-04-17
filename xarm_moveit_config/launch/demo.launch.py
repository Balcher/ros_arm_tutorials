from moveit_configs_utils import MoveItConfigsBuilder    # 用于加载Moveit配置如URDF、SRDF、规划组等，并构建MoveItConfigs对象
from moveit_configs_utils.launches import generate_demo_launch # moveit_configs_utils 提供的标准启动函数，用于生成MoveIt演示环境，RViz+运动规划

# xarm机器人名称，xarm_moveit_config 功能包
# to_moveit_configs 加载并返回MoveITConfigs对象，包含
# 1. 机器人的URDF文件（moveit_config.robot_description）
# 2. SRDF（moveit_config.robot_description_semantic）
# 3. 规划组（moveit_config.robot_description_kinematics）
# 4. 关节限制、传感器配置等
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("xarm", package_name="xarm_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)

# generate_demo_launch(moveit_config) 使用moveit_config生成标准的MoveIt演示启动文件，通常包含：
# 1. RViz 可视化
# 2. MoveGroup(运动规划服务)
# 3. FakeController 模拟执行
# 4. PlanningScene 场景管理