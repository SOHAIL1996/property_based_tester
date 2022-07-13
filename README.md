# Property-Based Testing: Formalized Robotic Testing for Standard Compliance

### Author 

*Salman Omar Sohail*

## Framework Overview

![Overview Property-Based Testing: Formalized Robotic Testing for Standard Compliance](https://github.com/SOHAIL1996/property_based_tester/blob/main/documentation/Figures/RevisedPlannedSafetyFrameWork-1.jpg)

## Running Framework

1. ./atg.sh 
2. python3 -m pytest --alluredir=results tests/nav_test.py -v -s
3. roslaunch jackal_navigation odom_navigation_demo.launch 
4. allure serve results/

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

## Hardware Requirements

These constitute the bare minimum requirements to run this package.

* `8 Gb ram`
* `Intel® Core™ i5-6300HQ CPU @ 2.30GHz × 4 `
* `Nvidia GeForce GTX 960M/PCIe/SSE2`
* `250 Gb hard disk`
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