# car_nav2
Using ROS2 Jazzy and Gazebo Harmonic for autonomous navigation simulation of a robot car.

This project is based on the BME MOGI - ROS course, with the camera part removed, and the mapping and navigation features retained. For further learning, please refer to https://github.com/MOGI-ROS.

This project is for personal learning purposes only. If there are any copyright infringements, please contact us for removal.

You need to do this:

1.Run the following command to clone the car_nav2 repository to your local machine:

    git clone https://github.com/2024828/car_nav2.git

2.The file uses some Gazebo models, which you can download from:https://drive.google.com/file/d/1tcfoLFReEW1XNHPUAeLpIz2iZXqQBvo_/view.

If you want to learn more about Gazebo models, please visit:https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics.

Make sure to let Gazebo know about their location by running:

    export GZ_SIM_RESOURCE_PATH=~/gazebo_models

3.Download a package in the `car_nav2` folder that can enable the recording of the robot's motion path.
    
    git clone https://github.com/MOGI-ROS/mogi_trajectory_server

If you don't need this, you can comment out the line  `launchDescriptionObject.add_action(trajectory_node)`  at the end of the `navigation_with_slam.launch.py` file.

4.Build the package and run:
    
    colcon build
    .install/setup.bash
    ros2 launch car_nav2 spawn_robot.launch.py

Open a new terminal and run:
    
    export GZ_SIM_RESOURCE_PATH=~/gazebo_models
    .install/setup.bash
    ros2 launch car_nav2 navigation_with_slam.launch.py

You can use the `2D goal pose` in the RViz2 toolbar to control the robot's movement and mapping. The robot will automatically plan the path.
