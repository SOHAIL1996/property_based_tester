# Property-Based Testing: Formalized Robotic Testing for Standard Compliance

![Jackal ISO 23482-1 Compliance Test](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/iso-23482-1_11.2.gif)

### Copyright and licence

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (C) 2022 by [Hochschule Bonn Rhein Sieg](https://www.h-brs.de/de)

### Author 

*Salman Omar Sohail*

### Co-authors

*Prof. Dr. Nico Hochgeschwender, Prof. Dr. Paul G. Pl ̈oger, Sven Schneider*

## Framework Overview

![Overview Property-Based Testing: Formalized Robotic Testing for Standard Compliance](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/figures/Methodology.jpg)


## Running Framework (Hypothesis|ROS|Gazebo|Allure|TextX)

1. Basic setup 
```bash
./atg.sh
``` 
2. Run from inside the `src/property_based_tester` folder, this applies the tests
```bash
python3 -m pytest --alluredir=results tests/ugv_test.py -v -W ignore::DeprecationWarning --count=2 --repeat-scope=class
python3 -m pytest --alluredir=results tests/pblg_test_interface.py -v -W ignore::DeprecationWarning 
```

3. Run for the navigation tests
```bash
roslaunch jackal_navigation odom_navigation_demo.launch 
roslaunch husky_navigation move_base_mapless_demo.launch 
```

4. World configuration inside the `src/property_based_tester/configuration/property_based_tester_params.yaml`

Current available parameters:
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

![Husky Odom Test](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/gifs/husky_waypoint_rviz.gif)

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

- (In the works!)  Used for feeding into hypothesis for  generating tests 

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
* `yocs_cmd_vel_mux package`
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

* `Jackal`
* `xARM6`
* `Husky`

## Hardware Requirements

These constitute the bare minimum requirements to run this package.

* `16 Gb RAM`
* `AMD Ryzen 5 5600H CPU @ 3.30GHz`
* `Nvidia GeForce RTX 3060`
* `250 Gb SSD`
<!-- 

## Note has to be redone


## Setup

1. git clone and Install the Toyota HSR package from gitlab in the `catkin_ws/src/`.
2. git clone and Install the MAS domestic repository package from github in the `catkin_ws/src/`.
3. git clone and Install the MAS HSR package from gitlab in the `catkin_ws/src/`.
4. Build the catkin_ws.

## First time installation

- Correct directory of world file.

## Settings
- Add to bash.rc file `export ROBOT_ENV=atg_lab`
- Add the map folder to `mdr_environments` which should contain the `map.yaml`,`map.pgm` and `navigations_goal.yaml` files.

### Information
After setting up the Toyota HSR environment. You will have to source the `atg` package and it is best to add it
in the `~/.bashrc` below the ros kinetic package.


## Configuration

To configure the parameters of the simulator, open the utilities folder and set the parameters in the configuration file.

## Simulation Startup

To use simply open the simulator with Lucy in it, run `./atg.sh` from `$(Parent directory)/atg`.

## Running Navigation Test

- Run `./atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/nav_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/nav_test-1.png)

## Running Perception Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/perceive_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/perceive_test-1.png)

## Running Pick Action Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/pick_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/pick_test-1.png)


## Running Complex Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/complex_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/complex_scenario-1.png) -->
