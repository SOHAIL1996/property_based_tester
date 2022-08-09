# Buidling the mimic plugin for xARM6

- Build the gazebo plugin

https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins

```bash
mkdir build && cd build
cmake ..
make
```
- In .bashrc file add

```bash
export GAZEBO_PLUGIN_PATH=/home/sorox23/robotic_ws/master_thesis_ws/src/property_based_tester/plugins/roboticsgroup_gazebo_plugins/build/devel/lib
```