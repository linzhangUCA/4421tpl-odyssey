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

**NOTE**: you will work with a highly integrated system, any subtle malfunction could fail the entire system.
Check your wire connections and battery health, regularly. 
Do simple unit tests if anything is not functional. 
Take notes for things you cannot memorize.
And don't be discouraged to start everything over.

## Requirements

### (45%) ROS Package
- Create or reuse a ROS package to host following modules. 
Include/Upload the pakcage in this repository.
- Grader(s) will download your package to a computer with ROS 2 installed.
  Your package will be built and verified if all the executables and launch files are functional or not.
  
1. **(11%)** Develope a **`hardware_interface`** node for controlling and monitoring the robot. 
    - (4%) Publish **`/odom`** topic with `nav_msgs/msg/Odometry` message according to robot's actual velocity received from Pico.
    - (4%) Broadcast transformation from **`odom`** frame to **`base_link`** frame with `geometry_msgs/msg/TransformStamped` message.
    - (3%) Subscribe to `/cmd_vel` topic, send robot's target velocity to Pico use values embedded in the `geometry_msgs/msg/Twist` message.   
    > Run this node on **Raspberry Pi**.
2. **(8%)** Develop a launch file: **`bringup_driver.launch.py`**, to get the robot ready for SLAM.
    - (2%) Broadcast reasonable static transformation from `base_link` to `base_footprint`.
    - (2%) Broadcast reasonable static transformation from `base_link` to `lidar_link`.
    - (2%) Start `rplidar_ros` package's `rplidar_node` node with its `frame_id` parameter set to `'lidar_link'` and `angle_compensate` parameter set to `True`.
    - (2%) Start `hardware_interface` node.
    > Launch this file on **Raspberry Pi**.
3. **(9%)** Prepare the configuration files in `<your package path>/configs/` directory.
    - (2%) a reasonable **`mapping_configs.yaml`** file for `slam_toolbox`package's `online_async_launch.py`.
      - `String value is: mapping` is expected by checking the parameter value: `ros2 param get /slam_toolbox mode`.
    - (3%) a reasonable **`localization_configs.yaml`** file for `slam_toolbox` package's `localization_launch.py`.
      - `String value is: localization` is expected by checking the parameter value: `ros2 param get /slam_toolbox mode`.
      - `String value is: /home/<user name>/<ros workspace name>/src/<project name>/<package name>/maps/final_map` is expected by checking the parameter value: `ros2 param get /slam_toolbox map_file_name`.
    - (4%) a reasonable **`nav_configs.yaml`** file for `nav2_bringup` package's `navigation_launch.py`.
    - (optional) a reasonable `gamepad_config.yaml` file for `teleop_twist_joy` package's `teleop-launch.py`.
    > You'll want to tweak these configuration files to optimize your robot's performance.
4. **(5%)** Develop a launch file: **`create_map.launch.py`** , to map interested area.
    - (2%) Launch `teleop_launch.py` from `teleop_twist_joy` package.
    - (2%) Launch `online_async_launch.py` from `slam_toolbox` package with:
        - `use_sim_time` set to `false`.
        - `slam_params_file` set to a customed `mapping_configs.yaml`.
    - (1%) Start `rviz2`.
    > Launch this file on **your laptop**.
5. **(5%)** Save/Serialize the map and save the map to **`<your package path>/maps/final_map.data`** and **`<your package path>/maps/final_map.posegraph`**
6. **(5%)** Develop a launch file: **`final_navigation.launch.py`**, for autonomous navigation.
    - (2%) Launch `localization_launch.py` from `slam_toolbox` package with:
        - `use_sim_time` set to `false`.
        - `slam_params_file` set to a customed `localization_configs.yaml`.
    - (2%) Launch `navigation_launch.py` from `nav2_bringup` package with:
        - `use_sim_time` set to `false`.
        - `params_file` set to a customed `nav_configs.yaml`.
    - (1%) Start `rviz2`.
    > Launch this file on **your laptop**.
7. **(2%)** Fill correct information in `package.xml` and `setup.py`.
   
#### Hints
- Refer to [`homer_control`](https://github.com/linzhangUCA/homer/tree/main/homer_control) package.
  You can find examples for every step.
- To publish `/odom` topic and broadcast tf from `odom` to `base_link`, you'll need to calculate the robot's pose and read its velocity to fill the `nav_msgs/msg/Odometry` and `geometry_msgs/msg/TransformStamped` message.
  Refer to [Assignment 3](https://classroom.github.com/a/R9LNWs9-) and [Assignment 5](https://classroom.github.com/a/cGOzC79L).
- Look for parameters related to `radius` and `vel` in **`nav_configs.yaml`**, tweaking them to change behavior of your robot.
- Read ROS [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html) about **Launch** if you feel difficult to get the launch files managed.
- [HomeR's documentation](https://linzhanguca.github.io/homer/) site could be helpful.
- [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) made a series of great videos to teach you how to:
  - [Easy SLAM with ROS using slam_toolbox](https://www.youtube.com/watch?v=ZaiA3hWaRzE)
  - [Making robot navigation easy with Nav2 and ROS!](https://www.youtube.com/watch?v=jkoGkAd0GYk)
  
### (25%) Document the project in the [Documentation](README.md#documentation) section.
1. (5%) Illustrate a Schematic of mechanical design with specific dimensions and locations of key components.
2. (5%) Illustrate a Wiring diagram for the relationships among the batteries, motors, motor driver, Pico board, power management board and Raspberry Pi.
   Please mark/denote the signal wires and power wires.
3. (5%) Illustrate a graph of ROS Nodes with all participating/active nodes and topics.
4. (10%) Search and read the SLAM algorithm you've been used in this project.
   Imagine your readers are engineering major freshmen.
   State following with your own words and math expressions in [SLAM Approach](README.md#slam-approach) section:
   1. List of key concepts, algorithms, functionalities.
   2. Descriptions of the mechanism/process of the key features. 
     
### (30%) Demonstration
Deliver a cup of coffee to PAE department office using your robot.
You are expected to demonstrate the robot not only to the people from Annex 105, but also to anyone who may show up on the hallway of Lewis Science Center.
Your demonstration should include two parts.
1. (6%) 5-minute introduction to your project include:
   - (1%) Your team
   - (2%) Background and objectives
   - (3%) Methodology
2. (24%) Live demo on coffee delivery.
   - You can attempt 3 times (from the start point)
   - During any attempt, you set your goal 5 times maximum by cliking the `2D Goal Pose` button in Rviz2

## Documentation

### Mechanical Design
![]()

### Wiring Diagram
![]()

### Node Graph
![]()

### SLAM Approach


