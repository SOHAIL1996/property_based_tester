# Property-Based Testing: Formalized Robotic Testing for Standard Compliance

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/iso-23482-1_11.2.gif" width="100%"/>
</p>

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec11.1.gif" width="32%"/>
  &nbsp;
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec13.2.gif" width="32%"/>
  &nbsp; 
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec13.3.gif" width="32%"/>
</p>

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec15.1.gif" width="32%"/>
  &nbsp;
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec15.3.gif" width="32%"/>
  &nbsp; 
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec15.4.gif" width="32%"/>
</p>

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec16.gif" width="32%"/>
  &nbsp;
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_iso23482-1_sec17.gif" width="32%"/>
  &nbsp; 
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/rovo_iso23482-1_sec11.gif" width="32%"/>
</p>

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/b1_iso23482-1_sec12.gif" width="32%"/>
  &nbsp;
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/xarm6_iso23482-1_sec17.gif" width="32%"/>
  &nbsp; 
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/b1_iso23482-1_sec13.gif" width="32%"/>
</p>

### Copyright and licence

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (C) 2022 by [Hochschule Bonn Rhein Sieg](https://www.h-brs.de/de)

### Author 

*Salman Omar Sohail*

### Co-authors

*Prof. Dr. Nico Hochgeschwender, Prof. Dr. Paul G. Pl ̈oger, Sven Schneider*

## Framework Overview

![Overview Property-Based Testing: Formalized Robotic Testing for Standard Compliance](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/figures/Methodology.jpg)


## Code Documentation

[Docs: Property-Based Testing: Formalized Robotic Testing for Standard Compliance](https://htmlpreview.github.io/?https://github.com/SOHAIL1996/property_based_tester/blob/main/sphinx/_build/html/index.html)

## Running Framework (Hypothesis|ROS|Gazebo|Allure|TextX)

1. Basic setup 
```bash
./atg.sh
``` 
2. Run from inside the `src/property_based_tester` folder, this applies the tests
```bash
python3 -m pytest --alluredir=results tests/pbt_mobile_robot.py -v -W ignore::DeprecationWarning 
python3 -m pytest --alluredir=results tests/pbt_robotic_arm.py -v -W ignore::DeprecationWarning 
python3 -m pytest --alluredir=results tests/pbt_random.py -v -W ignore::DeprecationWarning 
```
  - The first test is for any ground vehicle
  - The second test is for robotic manipulators
  - The third test is for randomized testing

**_NOTE:_** Examples for applying these tests are available in `src/property_based_tester/
robot_test_definition_language/*_test_definitions`

3. Run for the navigation tests
```bash
roslaunch jackal_navigation odom_navigation_demo.launch 
roslaunch husky_navigation move_base_mapless_demo.launch 
```

4. Robot configuration inside the `src/property_based_tester/configuration/property_based_tester_params.yaml`

Current available robots:
```bash
 Robot: 
   robot_urdf_name: husky # URDF name
   robot_spawner_name: spawn_husky_controller.launch
   robot_size: [0.580, 0.430, 0.250] # x,y,z in meters
```

```bash
 Robot: 
   robot_urdf_name: jackal_robot_issac # URDF name
   robot_spawner_name: spawn_jackal_controller.launch
   robot_size: [0.990, 0.670, 0.390] # x,y,z in meters
```

```bash
 Robot: 
   robot_urdf_name: xarm6_gripper
   robot_spawner_name: spawn_xarm6_controller.launch
```

```bash
 Robot: 
   robot_urdf_name: rovo # URDF name
   robot_spawner_name: spawn_rovo_wo_controller.launch
   robot_controller: rovo_standalone_controller.launch
   robot_velocity: /rovo_velocity_controller/cmd_vel
   robot_size: [1.200, 1.230, 0.530] # x,y,z in meters
```

```bash
 Robot: 
   robot_urdf_name: b1 # URDF name
   robot_spawner_name: spawn_b1_wo_controller.launch
   robot_controller: b1_standalone_controller.launch
   robot_velocity: /b1/cmd_vel/smooth
   robot_size: [1.10, 0.45, 0.50] # x,y,z in meters
```


<p align="center">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_waypoint_rviz.gif" />
</p>

### Result generation (Allure)

- Run from inside the `src/property_based_tester` folder, this generates the results viewable from chrome

```bash
./result_generation.sh
```
### Collision sensor addition in Gazebo

1. Use the default template to add the senor plugin to your robot.

- Link and joint template:
```
<!-- Links and joints templplate if you do not already have -->
 <link name="sim_collider">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 0.8 0.1"/>
      </geometry>
    </collision>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
         <box size="2.0 0.8 0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <joint name="sim_collider_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/> <!-- Change param parent link -->
    <child link="sim_collider"/>   <!-- Change param same as link -->
  </joint>
```

- Sensor template:
```
<!-- Contact Sensor -->
  <gazebo reference="sim_collider"> <!-- Change param same as link -->
    <sensor name="sim_collider_link_contact_sensor" type="contact">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <contact>
        <collision>base_link_fixed_joint_lump__sim_collider_collision_2</collision> <!-- Change param same as link -->
      </contact>
      <plugin filename="libgazebo_ros_bumper.so" name="gazebo_ros_sim_collider_controller">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <frameName>world</frameName> <!-- Change param -->
        <bumperTopicName>bumper_contact_state</bumperTopicName>
      </plugin>
    </sensor>
    <material>Gazebo/Orange</material>
  </gazebo>
```
2. Convert your URDF to SDF as there are parsing issues in detecting collision name.
```bash
gz sdf -p my.urdf > my.sdf
```
- Incase you have to convert your xacro urdf to URDF

```bash
rosrun xacro xacro -o model.urdf model.urdf.xacro
```

3. Search in the .sdf file the collision tag corresponding to your URDF and place it in the contact sensors collision tag (i.e. sim_collider).

### Property-Based Language Generator (TextX)

- Run from inside the `src/property_based_tester/property_based_language_generator` folder, generates a DSL

```bash
./pblg.sh
```

- Test definitions are written in `src/property_based_tester/property_based_language_generator/standard_test_definitions.pblg`

- Used for feeding into hypothesis for  generating tests 

![Sample: TextX Generation](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/figures/standard_test_definitions.png)

## Running Framework (Hypothesis|ROS|Omniverse Nvidia IssacSim|Allure|TextX)

- (In the works!) Generate Universal Scene Description physics based simulation in ISSAC Sim.

```bash
./omni.sh
```
![Sample: Nvidia Issac Sim](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/figures/jackal_nvidia_issac_sim.png)

## Acknowledgements

Supervised by:
 - Prof. Dr. Nico Hochgeschwender
 - Prof. Dr. Paul G. Pl ̈oger
 - Sven Schneider

## Software Requirements

* `Ubuntu 20.04 LTS`
* `Python 3.6.12 64-bit`
* `Gazebo 7.16.1`
* `Catkin-pkg 0.4.22-100`
* `ROS-Noetic`
* `numpy 1.11.0`
* `numpy-stl`
* `cuda 11.0`
* `cuddnn 8`
* `tensorflow 1.4.0`
* `keras 2.0.8`
* `pandas 0.17.1`
* `termcolor 1.1.0`
* `pytest==4.6.11`
* `maven`
* `jdk 8`
* `allure-pytest==2.6.0`
* `allure==2.6.0`
* `torch==1.4.0`
* `torchvision==0.5.0`
* `allure-python-commons==2.6.0`
* `hypothesis--4.57.1`
* `Nvidia-Issac-Sim`
* `TextX`

### Required ROS Packages

* [Husky](https://github.com/husky/husky)
* [Jackal](https://github.com/jackal/jackal)
* [xARM6](https://github.com/xArm-Developer/xarm_ros)
* [Ouster](https://github.com/ouster-lidar/ouster-ros)
* [Gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)
* `yocs_cmd_vel_mux package`
* `MBS ROVO2`
* `RoboticsGroup_gazebo_plugins`

## Hardware Requirements

These constitute the bare minimum requirements to run this package.

* `16 Gb RAM`
* `AMD Ryzen 5 5600H CPU @ 3.30GHz`
* `Nvidia GeForce RTX 3060`
* `250 Gb SSD`

# Issues

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/rovo_physics_simulator_bug.gif" width="100%"/>
</p>

## Movebase not working

The movebase is not running. Make sure the topics are correct for the odom, map, and cmd_vel. Also,
make sure that sim_time is enabled for move_basei.e.:
```bash
<param name="use_sim_time" value="true" />
```

## Ouster or other Gazebo plugins not working

Add this and place the plugins binaries and shared objects into this folder

```bash
export GAZEBO_PLUGIN_PATH=/home/sorox23/robotic_ws/master_thesis_ws/src/property_based_tester/plugins/gazebo_plugins_manual_amd64
```