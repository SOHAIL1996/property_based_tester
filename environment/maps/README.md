# Mapping Procedure

1. Run the `atg.sh`.
2. Run `rosnode kill /pose_integrator`.
3. Run `rosrun hector_mapping hector_mapping _map_size:=2048 _map_resolution:=0.05 _pub_map_odom_transform:=true _scan_topic:=/hsrb/base_scan _use_tf_scan_transformation:=true _map_update_angle_thresh:=2.0 _map_update_distance_thresh:=0.10 _scan_subscriber_queue_size:=1 _update_factor_free:=0.39 _update_factor_occupied:=0.85 _base_frame:=base_link`
4. Run `rosrun rviz rviz` and load the Mapper.rviz file in the maps folder.
5. Run `rqt`.
6. Goto `Plugins -> Tools -> Robot Steering` and change the velocity command to `/hsrb/command_velocity`.
7. Save the map by `rosrun map_server map_saver -f my_map`.
8. Goto the `mdr_environments` package and place the map in the `atg_lab` folder. Additionally, place the map in this maps folder.
9. Update the `navigation` file in `mdr_enviroments` by utilizing the `2D nav` coordinates given by `rviz` in the terminal.