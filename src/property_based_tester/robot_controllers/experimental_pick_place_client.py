#!/bin/usr/env python2
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
"""
Experimental pick, place client Toyota HSR: The client activates the pick and place action based
on the default Toyota HSR client.
"""
import math

'''Depreciated Libraries'''
# import hsrb_interface
# from hsrb_interface import geometry
# import tmc_interactive_grasp_planner.srv


def MoveItPickAndPlace(pick_x = 0, pick_y = 0, pick_z = 0, place_x = 0, place_y = 0, place_z = 0):
    """Provides cartesian goal commands to the Toyota HSR to perform pick and place actions in 
    the map frame

    Args:
        pick_x (int, optional): x-coordinates for pick action. Defaults to 0.
        pick_y (int, optional): y-coordinates for pick action. Defaults to 0.
        pick_z (int, optional): z-coordinates for pick action. Defaults to 0.

        place_x (int, optional): x-coordinates for place action. Defaults to 0.
        place_y (int, optional): y-coordinates for place action. Defaults to 0.
        place_z (int, optional): z-coordinates for place action. Defaults to 0.
    """
    # Prepare hsrb_interface
    robot = hsrb_interface.Robot()
    whole_body = robot.get("whole_body")
    omni_base = robot.get("omni_base")
    gripper = robot.get("gripper")
    
    whole_body.collision_world  = hsrb_interface.collision_world.CollisionWorld('global_collision_world')
    whole_body.collision_world.add_box(x=3, y=1.4, z=0.48, pose=geometry.pose(x=0,y=1, z= 0.25), frame_id='map')   
    print('\nMoving to neutral')
    whole_body.move_to_neutral()
    # whole_body.move_to_joint_positions({"head_tilt_joint": -0.3, "head_pan_joint": 0})
    # print('Navigating to designated')
    # omni_base.go_abs(0,0,1.5708)
    # print('Raising arm')
    # whole_body.move_to_joint_positions({'arm_lift_joint': 0.1})

    print('Moving to object')
    whole_body.move_end_effector_pose(geometry.pose(x=0, y=0.93, z=0.65, 
                                                    ei=math.radians(-180), ej=math.radians(-90), ek=0), 
                                                    ref_frame_id='odom')
    # gripper.command(0.0)

    # Approach
    # whole_body.move_end_effector_pose(grasp_pose, "odom")
    # Grasp
    # gripper.grasp(-0.01)
    # whole_body.move_to_neutral()