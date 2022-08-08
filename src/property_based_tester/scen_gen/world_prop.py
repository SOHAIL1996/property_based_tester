#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
World properties

Extracts information of the gazebo world.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""

import rospy
import numpy as np
from termcolor import colored

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetWorldProperties

def world_state():
    """It extracts the information of all models in the gazebo world.

    Returns:
        [list]: Returns list of all relevant information
    """        
    logs = [['Models', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4']]
    lucy_logs = [['Gripper Position', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4']]
    rospy.wait_for_service("/gazebo/get_world_properties")
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/get_link_state")
    try:
        # print(colored('Acquiring Model State','yellow'))
        
        world = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        end_of_effector_left = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        end_of_effector_right= rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        world_props = world()
        lucy_lef_gripper = end_of_effector_left('hsrb::hand_l_distal_link','world')
        lucy_rig_gripper = end_of_effector_right('hsrb::hand_r_distal_link','world')
        for objs in world_props.model_names:
            coordinates = model(objs,'world')
            temp_data   = [objs,np.around(coordinates.pose.position.x,3),
                            np.around(coordinates.pose.position.y,3),
                            np.around(coordinates.pose.position.z,3),
                            np.around(coordinates.pose.orientation.x,3),
                            np.around(coordinates.pose.orientation.y,3),
                            np.around(coordinates.pose.orientation.z,3),
                            np.around(coordinates.pose.orientation.w,3)]
            logs.append(temp_data)    
            

        lucy_data_lef  = ['Lucy left gripper',np.around(lucy_lef_gripper.link_state.pose.position.x,3),
                        np.around(lucy_lef_gripper.link_state.pose.position.y,3),
                        np.around(lucy_lef_gripper.link_state.pose.position.z,3),
                        np.around(lucy_lef_gripper.link_state.pose.orientation.x,3),
                        np.around(lucy_lef_gripper.link_state.pose.orientation.y,3),
                        np.around(lucy_lef_gripper.link_state.pose.orientation.z,3),
                        np.around(lucy_lef_gripper.link_state.pose.orientation.w,3)]
        
        lucy_data_rig  = ['Lucy right gripper',np.around(lucy_rig_gripper.link_state.pose.position.x,3),
                        np.around(lucy_rig_gripper.link_state.pose.position.y,3),
                        np.around(lucy_rig_gripper.link_state.pose.position.z,3),
                        np.around(lucy_rig_gripper.link_state.pose.orientation.x,3),
                        np.around(lucy_rig_gripper.link_state.pose.orientation.y,3),
                        np.around(lucy_rig_gripper.link_state.pose.orientation.z,3),
                        np.around(lucy_rig_gripper.link_state.pose.orientation.w,3)]
        
        lucy_logs.append(lucy_data_lef)
        lucy_logs.append(lucy_data_rig) 
        
        # print(colored('Successfully acquired Model State','green')) 
        return logs, lucy_logs
    except rospy.ServiceException as e:
        print(colored('Cannot acquire Model State.','red')) 
    return 

