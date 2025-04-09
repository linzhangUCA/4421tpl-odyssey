# Final Project: The Odyssey

## Intro
Welcome to your final (second) project. 
In this project, your robot is expected to navigate from an entrance of Lewis Science Center to the PAE Department office, **autonomously**. 
The starting point and destination is illustrated in the diagrams below.
To test the navigation is successful or not, your robot needs to deliver a cup of coffee to the front desk in the office. 

- The floor plan for the interested area in Lewis Science Center is shown below.
![path](figures/lsc_nav_floorplan.png)

- A roughly measured floor dimensions is illustrated below.
![dimensions](figures/lsc_nav_dimensions.png)

## Objectives
- Prepare autonomous navigation with [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).
- Practice ROS navigation stack with [Nav2](https://docs.nav2.org/).
- Verify delivery solutions.

## Requirements
- Please complete the contents required below and strictly follow the required file names.
- Grader(s) will download your package to a computer with ROS 2 installed.
  Your package will be built and verified if all the executables and launch files are functional or not.
  
### ROS Package

Create or reuse a ROS package to host following modules. 
Include/Upload the pakcage in this repository.

1. Develope a **`hardware_interface`** node for controlling and monitoring the robot. 
    - Publish **`/odom`** topic with `nav_msgs/msg/Odometry` message according to robot's actual velocity received from Pico.
    - Broadcast transformation from **`odom`** frame to **`base_link`** frame with `geometry_msgs/msg/TransformStamped` message.
    - Subscribe to `/cmd_vel` topic, send robot's target velocity to Pico use values embedded in the `geometry_msgs/msg/Twist` message.   
    > Run this node on **Raspberry Pi**.
2. Develop a launch file: **`bringup_driver.launch.py`**, to get the robot ready for SLAM.
    - Broadcast reasonable static transformation from `base_link` to `base_footprint`.
    - Broadcast reasonable static transformation from `base_link` to `lidar_link`.
    - Start `rplidar_ros` package's `rplidar_node` node with its `frame_id` parameter set to `'lidar_link'` and `angle_compensate` parameter set to `True`.
    - Start `hardware_interface` node.
    > Launch this file on **Raspberry Pi**.
3. Prepare the configuration files in `<your package path>/configs/` directory.
    - a reasonable **`mapping_configs.yaml`** file for `slam_toolbox`package's `online_async_launch.py`.
      - `String value is: mapping` is expected by checking the parameter value: `ros2 param get /slam_toolbox mode`.
    - a reasonable **`localization_configs.yaml`** file for `slam_toolbox` package's `localization_launch.py`.
      - `String value is: localization` is expected by checking the parameter value: `ros2 param get /slam_toolbox mode`.
      - `String value is: /home/<user name>/<ros workspace name>/src/<project name>/<package name>/maps/final_map` is expected by checking the parameter value: `ros2 param get /slam_toolbox map_file_name`.
    - a reasonable **`nav_configs.yaml`** file for `nav2_bringup` package's `navigation_launch.py`.
    - (optional) a reasonable `gamepad_config.yaml` file for `teleop_twist_joy` package's `teleop-launch.py`.
4. Develop a launch file: **`create_map.launch.py`** , to map interested area.
    - Launch `teleop_launch.py` from `teleop_twist_joy` package.
    - Launch `online_async_launch.py` from `slam_toolbox` package with:
        - `use_sim_time` set to `false`.
        - `slam_params_file` set to a customed `mapping_configs.yaml`.
    - Start `rviz2`.
    > Launch this file on **your laptop**.
5. Save/Serialize the map and save the map to **`<your package path>/maps/final_map.data`** and **`<your package path>/maps/final_map.posegraph`**
6. Develop a launch file: **`final_navigation.launch.py`**, for autonomous navigation.
    - Launch `localization_launch.py` from `slam_toolbox` package with:
        - `use_sim_time` set to `false`.
        - `slam_params_file` set to a customed `localization_configs.yaml`.
    - Launch `navigation_launch.py` from `nav2_bringup` package with:
        - `use_sim_time` set to `false`.
        - `params_file` set to a customed `nav_configs.yaml`.
    - Start `rviz2`.
    > Launch this file on **your laptop**.
7. Fill correct information in `package.xml` and `setup.py`.
   
#### Hints
- Refer to [`homer_control`](https://github.com/linzhangUCA/homer/tree/main/homer_control) package.
  You can find examples for every step.
- To publish `/odom` topic and broadcast tf from `odom` to `base_link`, you'll need to calculate the robot's pose and read its velocity to fill the `nav_msgs/msg/Odometry` and `geometry_msgs/msg/TransformStamped` message.
  Refer to [Assignment 3](https://classroom.github.com/a/R9LNWs9-) and [Assignment 5](https://classroom.github.com/a/cGOzC79L).
- Read ROS [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html) about **Launch** if you feel difficult to get the launch files managed.
- [HomeR's documentation](https://linzhanguca.github.io/homer/) site could be helpful.
- [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) made a series of great videos to teach you how to:
  - [Easy SLAM with ROS using slam_toolbox](https://www.youtube.com/watch?v=ZaiA3hWaRzE)
  - [Making robot navigation easy with Nav2 and ROS!](https://www.youtube.com/watch?v=jkoGkAd0GYk)
  
### (30%) Document the project in the [Documentation](README.md#documentation) section.
1. (5%) Illustrate a Schematic of mechanical design with specific dimensions and locations of key components.
2. (5%) Illustrate a Wiring diagram for the relationships among the batteries, motors, motor driver, Pico board, power management board and Raspberry Pi.
   Please mark/denote the signal wires and power wires.
3. (5%) Illustrate a graph of ROS Nodes with all participating/active nodes and topics.
4. (15%) Search and read the SLAM algorithm you've been used in this project.
   Imagine your readers are engineering major freshmen.
   Explain following approaches as concise and as possible:
   1. Localization algorithm with.
      1. name
      2. mechanism/process
   2. Mapping algorithm with.
      1. name
      2. mechanism/process
  > Math is welcome.
     
### Demonstration
You are expected to demonstrate the robot not only to the people from Annex 105, but also to anyone who may show up on the hallway of Lewis Science Center.
Your demonstration 
1. 5-minute introduction include:
   - Your team
   - Your robot
   - Background and objectives of this project
   - Methodology
2. Live demo on coffee delivery

## Documentation

### Mechanical Design
![]()

### Wiring Diagram
![]()

### Node Graph
![]()

### Localization Algorithm

### Mapping Algorithm

