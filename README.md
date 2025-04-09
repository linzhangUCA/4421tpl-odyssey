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
    - Broadcast transformation **from `odom` frame to `base_link` frame** with `geometry_msgs/msg/TransformStamped` message.
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
    - a reasonable **`localization_configs.yaml`** file for `slam_toolbox` package's `localization_launch.py`.
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
7. Fill correct metadata in `package.xml` and `setup.py`.
   
#### Hints
- Refer to `[homer_control](https://github.com/linzhangUCA/homer/tree/main/homer_control)` package. You can find examples for every step.
- To publish `/odom` topic and broadcast tf from `odom` to `base_link`, you'll need to calculate the robot's pose and read its velocity to fill the `nav_msgs/msg/Odometry` and `geometry_msgs/msg/TransformStamped` message.
  Refer to [Assignment 3](https://classroom.github.com/a/R9LNWs9-) and [Assignment 5](https://classroom.github.com/a/cGOzC79L).
- .
### Documentation
- Use this `README` file or create a separate markdown file or upload a pdf file for the documentation.
- Describe the project in concise words. 
- Have the documents well organized (break it down into sections). 
- Please include following contents in your documents.
    1. **(10%)** Update the old supportive documents with new configurations (wiring diagram, mechanical schematics, software workflow, etc.). If not changed, provide a link or a copy of the old files.
    2. **(18%)** Describe the approaches your team employed to map the world, to localize the robot, to plan the trajectories and to follow the trajectories(the more technical details the better).
    3. **(10%)** Read the [NSPE Code of Ethics for Engineers](https://www.nspe.org/resources/ethics/code-ethics), evaluate your project using the [Code](https://www.nspe.org/resources/ethics/code-ethics). **You may want to submit such a discussion individually.** 
    5. **(2%)** Summarize achievements you've made in this project (bulleted items are welcome). You can share any thoughs or interesting findings, or discuss the future of the applications here as well.



### 2.2 Trouble Shooting
- **Q: What if I don't have an IMU?**

A: Open the extended Kalman-Filter configuration file: [`ekf.yaml`](diffbot_navigation/config/ekf.yaml), comment or delete the `imu0` related contents.
- **Q: How do I save a map?**

A: In Rviz, click `Panels` > `Add New Panel`, then select `SlamToolboxPlugin`, then press `OK`. You'll find a new panel opened on the left of your Rviz window. Select the text box next to the `Serialize Map` button, input the **full path** of your map file name. For example: `/home/<USERNAME>/<ROS_WORKSPACE_NAME>/src/<PROJECT_REPOSITORY_NAME>/diffbot_navigation/map/serialized_playground`. You can obtain the [`map`](diffbot_navigation/map/) folder path in vscode by right-click it, then select `Copy Path`. When you paste the path to the `Serialize Map` text box, **DO NOT forget to append a map name**. 
- **Q: How do I load a map?**

A: Open [`slam_localize_params`](diffbot_navigation/config/slam_localize_params.yaml), find the `map_file_name:` parameter (around line 19), edit the path according to your saved map path and name. For example: `/home/<USERNAME>/<ROS_WORKSPACE_NAME>/src/<PROJECT_REPOSITORY_NAME>/diffbot_navigation/map/serialized_playground`.


