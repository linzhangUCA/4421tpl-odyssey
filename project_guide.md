# Final Project: The Odyssey

## Overview
In this project, your robot is expected to **autonomously** navigate in Lewis Science Center from room 159 (Dr. Chen's lab) to room 171 (PAE Department office). 
The starting point and destination is illustrated in the diagram below.
To test the navigation is successful or not, your robot needs to deliver a cup of coffee to the front desk in the office (Rm. 171). 

- The floor plan for the interested area in Lewis Science Center is shown below.

![odyssey_map](images/odyssey_map.png)

Coffee cup dimensions:

![cup_dimensions](https://www.thepapercupcompany.com/assets/images/double-wall-coffee-dimesions21.gif)

## Objectives
- Prepare autonomous navigation with [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).
- Practice ROS navigation stack with [Nav2](https://docs.nav2.org/).
- Deliver coffee (No spill, hopefully).

## Requirements

### 1. 3-Minute Presentation
1. Objectives and background
2. Navigation approaches
3. Coffee delivery solutions
4. Q & A.
   
### 2. Coffee Delivery Solutions
- Introduce mechanical design of your team's coffee delivery solution in [README](README.md).
  - Illustrate layout of key components. 
  - Upload sketches or techdraws with critical dimensions
- Provide an installation guide in [README](README.md).

> [!NOTE]
> **Bonus**:
> - Illustrate installation with graphical ways.

### 3. Software Usage Instructions
Assume a user is going to deploy the ROS 2 packages developed related to this project on a Raspberry Pi and a server computer with newly installed ROS 2 Jazzy (no HomeR packages, no `slam_toolbox`, no Nav2).
Please provide instructions for the key steps of:
- setting up related ROS 2 packages on the Raspberry Pi.
- setting up related ROS 2 packages on the server computer.
- starting the map creation.
- Saving the map.
- (**Tricky**) starting the autonomous navigation with the saved map.

> [!NOTE]
> **Bonus**:
> - Use CLI for map saving.

### (65%) 4. Navigation Node
- Map the intrested region using proper ROS 2 packages and configurations.
   - Upload map files to the project repository.
   - Log commands for mapping in [README](README.md). 
- Develop a ROS node (in a Python script)for navigating the robot to its destination.
- Config navigator with proper settings.
- Pack your ROS node(s) and configurations.
  Upload the package to the project repository.
  Indicate maintainer's name, email, package description, license information in proper files.
  Your package will be built on instructor's computer. 
- (Optional) Upload any software that is different from the HomeR collections. 

> [!NOTE]
> **Bonus**:
> - Use action in the navigation node.

### (30%) Demonstration
**Time: Thursday, Apr. 30th @ 11 AM - 1 PM**

You are expected to demonstrate the robot not only to the people from Annex 105, but also to anyone who may show up in the hallway of Lewis Science Center.
Your demonstration should include two parts.
1. (5%) 3-minute introduction to your robot include:
   - (1%) Your team
   - (2%) Background and objectives
   - (2%) Methodology
2. (10%) Live demo on coffee delivery.
   - You can attempt 3 times (from the start point)
   - During any attempt, you can set navigation goal 5 times maximum by cliking the `2D Goal Pose` button in Rviz2
3. (10%) Live demo on return of the robot.
   - You can attempt 3 times (from the start point)
   - During any attempt, you can set navigation goal 5 times maximum by cliking the `2D Goal Pose` button in Rviz2
3. (5%) Answer questions.
   Any audience may be interested. So, get familiar with your robot and be prepare to answer the questions.
   
You will work with a highly integrated system, any subtle malfunction could fail the entire system.
- Check your wire connections and battery health **regularly**. 
- Do simple **unit tests** if anything is not functional. 
- **Take notes** for things you cannot memorize.
- And **don't be discouraged** to start everything over.









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
1. (5%) Illustrate a schematic of mechanical design with specific dimensions and locations of key components.
2. (5%) Illustrate a wiring diagram for the relationships among the batteries, motors, motor driver, power management board, Raspberry Pi and Pico.
   Please mark/denote the signal wires and power wires.
3. (5%) Illustrate a graph of ROS Nodes with all participating/active nodes and topics.
> You can upload all the drawings and figures to the [images/](images/) directory.
4. (10%) Search and read the SLAM algorithm you've been used in this project.
   Imagine your readers are engineering major freshmen.
   State following with your own words and math expressions in [SLAM Approach](README.md#slam-approach) section:
   1. List of key concepts, algorithms, functionalities.
   2. Descriptions of the mechanism/process of the key features. 
     
### (30%) Demonstration
**Time: Thursday, May 1st @ 11 A.M.**

You are expected to demonstrate the robot not only to the people from Annex 105, but also to anyone who may show up in the hallway of Lewis Science Center.
Your demonstration should include two parts.
1. (5%) 3-minute introduction to your robot include:
   - (1%) Your team
   - (2%) Background and objectives
   - (2%) Methodology
2. (10%) Live demo on coffee delivery.
   - You can attempt 3 times (from the start point)
   - During any attempt, you can set navigation goal 5 times maximum by cliking the `2D Goal Pose` button in Rviz2
3. (10%) Live demo on return of the robot.
   - You can attempt 3 times (from the start point)
   - During any attempt, you can set navigation goal 5 times maximum by cliking the `2D Goal Pose` button in Rviz2
3. (5%) Answer questions.
   Any audience may be interested. So, get familiar with your robot and be prepare to answer the questions.
   
You will work with a highly integrated system, any subtle malfunction could fail the entire system.
- Check your wire connections and battery health **regularly**. 
- Do simple **unit tests** if anything is not functional. 
- **Take notes** for things you cannot memorize.
- And **don't be discouraged** to start everything over.

## Documentation

### Mechanical Design
![]()

### Wiring Diagram
![]()

### Node Graph
![]()

### SLAM Approach


