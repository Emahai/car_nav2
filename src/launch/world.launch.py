import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    # 声明一个启动参数，用于指定加载的 Gazebo 世界文件的名称
    # 参数名为 'world'，默认值为 'home.sdf'
    world_arg = DeclareLaunchArgument(
        'world', default_value='home.sdf',
        description='要加载的 Gazebo 世界文件的名称'
    )

    # 获取 car_nav2 和 ros_gz_sim 两个包的共享目录路径
    # 这些目录路径将用于访问所需的资源文件
    pkg_car_nav2 = get_package_share_directory('car_nav2')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 更新环境变量 GZ_SIM_RESOURCE_PATH
    # 添加 car_nav2 包中 worlds 文件夹的路径，以便 Gazebo 能找到世界文件
    gazebo_models_path = os.path.join(pkg_car_nav2, 'worlds')
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # 包含 Gazebo 的启动文件
    # 将通过 `gz_sim.launch.py` 启动 Gazebo 并传入启动参数
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            # 通过 'gz_args' 参数指定加载的世界文件路径，并设置 Gazebo 启动选项
            'gz_args': [PathJoinSubstitution([
                pkg_car_nav2,
                'worlds',
                LaunchConfiguration('world')  # 使用前面定义的 'world' 参数值
            ]),
            TextSubstitution(text=' -r -v -v1')],  # Gazebo 的启动参数，包含实时更新和详细日志输出
            'on_exit_shutdown': 'true'  # 指定在 Gazebo 退出时关闭进程
        }.items()
    )

    # 创建 LaunchDescription 对象，用于存储所有启动行为
    launchDescriptionObject = LaunchDescription()

    # 将前面定义的启动参数和 Gazebo 启动行为添加到启动描述中
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    # 返回最终的 LaunchDescription 对象
    return launchDescriptionObject
