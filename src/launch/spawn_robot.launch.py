import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 生成启动描述对象
def generate_launch_description():
    # 获取车载导航包的路径
    pkg_car_nav2 = get_package_share_directory('car_nav2')

    # 获取gazebo模型路径并设置环境变量
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_car_nav2)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # 声明启动参数，用于传入Gazebo世界文件名称
    world_arg = DeclareLaunchArgument(
        'world', default_value='home.sdf',
        description='要加载的Gazebo世界文件的名称'
    )

    # 声明启动参数，用于传入URDF模型文件名称
    model_arg = DeclareLaunchArgument(
        'model', default_value='mycar.urdf',
        description='要加载的URDF模型文件的名称'
    )

    # 声明启动参数，用于传入机器人的x坐标
    x_arg = DeclareLaunchArgument(
        'x', default_value='2.5',
        description='生成机器人时的x坐标'
    )

    # 声明启动参数，用于传入机器人的y坐标
    y_arg = DeclareLaunchArgument(
        'y', default_value='1.5',
        description='生成机器人时的y坐标'
    )

    # 声明启动参数，用于传入机器人的偏航角（Yaw角）
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='-1.5707',
        description='生成机器人时的偏航角'
    )

    # 声明启动参数，用于启用或禁用仿真时间
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='启用仿真时间的标志'
    )

    # 设置URDF或Xacro文件的路径
    urdf_file_path = PathJoinSubstitution([
        pkg_car_nav2,  # 包名
        "urdf",        # URDF文件夹
        LaunchConfiguration('model')  # URDF文件名称
    ])

    # 获取Gazebo桥接配置文件路径
    gz_bridge_params_path = os.path.join(
        get_package_share_directory('car_nav2'),
        'config',
        'gz_bridge.yaml'
    )

    # 引入Gazebo世界启动文件
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_car_nav2, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # 创建URDF模型节点，使用/世界/<世界名称>/create服务生成机器人
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mycar",  # 机器人名称
            "-topic", "robot_description",  # 机器人描述话题
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", "0.5", "-Y", LaunchConfiguration('yaw')  # 初始生成位置
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # 用于桥接/cmd_vel和/odom话题的Gazebo桥接节点
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'  # 配置文件路径
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # 发布机器人状态的节点，用于向tf发布机器人坐标变换信息
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),  # 通过xacro命令加载URDF描述
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),  # 坐标变换数据话题映射
            ('/tf_static', 'tf_static')  # 静态坐标变换数据话题映射
        ]
    )

    # 扩展卡尔曼滤波器（EKF）节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_car_nav2, 'config', 'ekf.yaml'),  # EKF参数配置文件
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # 创建启动描述对象，并将各个节点加入描述
    launchDescriptionObject = LaunchDescription()

    # 添加各个声明的参数和节点
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(ekf_node)

    # 返回启动描述对象
    return launchDescriptionObject
