
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
    EmitEvent,
    OpaqueFunction
)

from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def validate_algorithm_selection(context):
    """检查是否同时选择了多个控制算法。"""

    options = [
        'use_wall_follow',
        'use_obstacle_avoidance',
        'use_point_control',
        'use_wall_follow_optimized',
    ]

    selected = []

    for option in options:
        value = context.perform_substitution(
            LaunchConfiguration(option)
        )
        if value.lower() in ('true', '1', 'yes'):
            selected.append(option)

    if len(selected) > 1:
        raise RuntimeError(
            '只能同时启动一个控制算法！'
            f'当前选择：{selected}'
        )

    return []

def generate_launch_description():

    # ==========================================
    # 1. 获取功能包路径
    # ==========================================

    maze_pkg = get_package_share_directory('maze_control')

    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')

    tb3_description_pkg = get_package_share_directory(
        'turtlebot3_description'
    )

    gazebo_pkg = get_package_share_directory('gazebo_ros')


    # ==========================================
    # 2. 获取 World、模型和 RViz 配置路径
    # ==========================================

    world_path = os.path.join(
        maze_pkg,
        'worlds',
        's_shape_maze.world'
    )

    robot_model_path = os.path.join(
        tb3_pkg,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    # TurtleBot3 Burger 的 URDF 描述文件
    robot_urdf_path = os.path.join(
        tb3_description_pkg,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    # RViz2 配置文件
    rviz_config_path = os.path.join(
        maze_pkg,
        'rviz',
        'maze.rviz'
    )


    # ==========================================
    # 3. 配置 Gazebo 模型搜索路径
    # ==========================================

    gazebo_model_path = os.path.join(
        tb3_pkg,
        'models'
    )

    old_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')

    if old_model_path:
        gazebo_model_path += os.pathsep + old_model_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )


    # ==========================================
    # 4. 设置启动参数
    # ==========================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Path to Gazebo world file'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Robot initial X position'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='1.0',
        description='Robot initial Y position'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.02',
        description='Robot initial Z position'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Robot initial yaw angle'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )
        # 算法节点启动选项，默认全部关闭
    use_wall_follow_arg = DeclareLaunchArgument(
        'use_wall_follow',
        default_value='false',
        description='Start left wall following algorithm'
    )

    use_obstacle_avoidance_arg = DeclareLaunchArgument(
        'use_obstacle_avoidance',
        default_value='false',
        description='Start obstacle avoidance algorithm'
    )

    use_point_control_arg = DeclareLaunchArgument(
        'use_point_control',
        default_value='false',
        description='Start point-to-point control algorithm'
    )

    use_wall_follow_optimized_arg = DeclareLaunchArgument(
        'use_wall_follow_optimized',
        default_value='false',
        description='Start optimized left wall following algorithm'
    )


    # ==========================================
    # 5. 启动 Gazebo 并加载迷宫
    # ==========================================

    gazebo_launch_path = os.path.join(
        gazebo_pkg,
        'launch',
        'gazebo.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),

        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )


    # ==========================================
    # 6. 启动 robot_state_publisher
    # ==========================================

    # 使用 xacro 解析 URDF，生成机器人描述参数
    robot_description = Command([
        'xacro ',
        robot_urdf_path
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        name='robot_state_publisher',

        output='screen',

        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )


    # ==========================================
    # 7. 在 Gazebo 中生成 TurtleBot3 Burger
    # ==========================================

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',

        arguments=[
            '-entity', 'turtlebot3_burger',

            '-file', robot_model_path,

            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],

        output='screen'
    )

    # 等待 Gazebo 完成初始化后生成机器人
    delayed_spawn_robot = TimerAction(
        period=8.0,
        actions=[spawn_robot]
    )


    # ==========================================
    # 8. 启动 RViz2 可视化
    # ==========================================
    # 左墙跟随算法节点
    wall_follow_node = Node(
        package='maze_control',
        executable='wall_follow',
        name='wall_follow',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('use_wall_follow'))
    )

    # 反应式避障算法节点
    obstacle_avoidance_node = Node(
        package='maze_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(
            LaunchConfiguration('use_obstacle_avoidance')
        )
    )

    # 点到点控制算法节点
    point_control_node = Node(
        package='maze_control',
        executable='point_control',
        name='point_control',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('use_point_control'))
    )

    # 优化版左墙跟随算法节点
    wall_follow_optimized_node = Node(
        package='maze_control',
        executable='wall_follow_optimized',
        name='wall_follow_optimized',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(
            LaunchConfiguration('use_wall_follow_optimized')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',

        name='rviz2',

        arguments=[
            '-d', rviz_config_path
        ],

        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],

        output='screen'
    )


    # ==========================================
    # 9. 返回 Launch 描述
    # ==========================================
    return LaunchDescription([
        set_gazebo_model_path,

        # Launch 参数
        world_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        use_sim_time_arg,
        use_wall_follow_arg,
        use_obstacle_avoidance_arg,
        use_point_control_arg,
        use_wall_follow_optimized_arg,
        OpaqueFunction(function=validate_algorithm_selection),

        # 仿真环境与机器人
        gazebo,
        robot_state_publisher,
        delayed_spawn_robot,

        # 可选算法节点
        wall_follow_node,
        obstacle_avoidance_node,
        point_control_node,
        wall_follow_optimized_node,

        # RViz2
        rviz
    ])