import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 生成启动描述对象
def generate_launch_description():

    # 获取car_nav2包的共享目录路径
    pkg_car_nav2 = get_package_share_directory('car_nav2')

    # 获取Gazebo模型路径，并将其添加到GZ_SIM_RESOURCE_PATH环境变量中
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_car_nav2)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # 声明一个参数，用于控制是否启动RViz
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz, default is true'
    )

    # 声明一个参数，用于指定RViz配置文件
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='Specify the RViz configuration file'
    )

    # 声明一个参数，用于启用仿真时间
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable simulation time, default is True'
    )

    # 生成交互式Marker配置文件的路径
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'  # 配置文件路径
    )

    # 获取Nav2导航启动文件的路径
    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'  # Nav2导航启动文件路径
    )

    # 获取导航参数配置文件的路径
    navigation_params_path = os.path.join(
        get_package_share_directory('car_nav2'),
        'config',
        'navigation.yaml'  # 导航参数配置文件
    )

    # 获取slam_toolbox参数配置文件的路径
    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('car_nav2'),
        'config',
        'slam_toolbox_mapping.yaml'  # SLAM工具箱的配置文件
    )

    # 定义RViz启动节点
    rviz_node = Node(
        package='rviz2',  # 使用rviz2包
        executable='rviz2',  # 启动rviz2执行文件
        arguments=['-d', PathJoinSubstitution([pkg_car_nav2, 'rviz', LaunchConfiguration('rviz_config')])],  # 加载指定的配置文件
        condition=IfCondition(LaunchConfiguration('rviz')),  # 如果'rviz'参数为True，则启动RViz
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},  # 设置使用仿真时间
        ]
    )

    # 定义交互式Marker服务器节点，允许通过交互式控制机器人
    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',  # 使用交互式Marker服务器包
        executable='marker_server',  # 启动Marker服务器执行文件
        name='twist_server_node',  # 节点名称
        parameters=[interactive_marker_config_file_path],  # 使用指定的配置文件
        output='screen',  # 将输出打印到控制台
    )

    # 获取SLAM工具箱的启动文件路径
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'  # SLAM工具箱的异步启动文件
    )

    # 定义SLAM工具箱的启动节点
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),  # 引入SLAM工具箱的启动文件
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),  # 使用仿真时间
            'slam_params_file': slam_toolbox_params_path,  # 使用SLAM工具箱的配置文件
        }.items()
    )

    # 定义Nav2导航的启动节点
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),  # 引入Nav2导航的启动文件
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),  # 使用仿真时间
            'params_file': navigation_params_path,  # 使用导航参数配置文件
        }.items()
    )

    # 创建一个启动描述对象
    launchDescriptionObject = LaunchDescription()

    # 添加启动参数和节点到启动描述对象中
    launchDescriptionObject.add_action(rviz_launch_arg)  # 添加是否启动RViz的参数
    launchDescriptionObject.add_action(rviz_config_arg)  # 添加RViz配置文件参数
    launchDescriptionObject.add_action(sim_time_arg)  # 添加仿真时间参数
    launchDescriptionObject.add_action(rviz_node)  # 添加启动RViz的节点
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)  # 添加交互式Marker服务器节点
    launchDescriptionObject.add_action(slam_toolbox_launch)  # 添加SLAM工具箱启动节点
    launchDescriptionObject.add_action(navigation_launch)  # 添加Nav2导航启动节点

    # 返回启动描述对象
    return launchDescriptionObject
