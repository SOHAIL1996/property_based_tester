#!/usr/bin/env python
""" 
Place client: The client activates the MDR place action module.
"""
import rospy
import actionlib
import numpy as np

'''Depreciated Libraries'''
# from mdr_place_action.msg import PlaceAction, PlaceGoal

def placer_client(x, y, z, R,P,Y):
    """Initialized place action for the Toyota HSR using the MDR repository of MAS.

    Args:
        x (float): x-coordinates for place action
        y (float): y-coordinates for place action
        z (float): z-coordinates for place action

        R (float): r-orientation for place action
        P (float): p-orientation for place action
        Y (float): y-orientation for place action

    Returns:
        bool: Returns True if action commands were successful.
    """
    rospy.init_node('place_action_client_test')

    client = actionlib.SimpleActionClient('place_server', PlaceAction)
    client.wait_for_server()

    goal = PlaceGoal()
    goal.pose.header.frame_id = 'base_link'
    goal.pose.header.stamp = rospy.Time.now()

    # goal.pose.pose.position.x = 0.418
    # goal.pose.pose.position.y = 0.078
    # goal.pose.pose.position.z = 0.842

    # goal.pose.pose.orientation.x = 0.758
    # goal.pose.pose.orientation.y = 0.000
    # goal.pose.pose.orientation.z = 0.652
    # goal.pose.pose.orientation.w = 0.000
    
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.position.z = z

    q1     = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
    q2     = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
    q3     = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
    q4     = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)

    goal.pose.pose.orientation.x = q1
    goal.pose.pose.orientation.y = q2
    goal.pose.pose.orientation.z = q3
    goal.pose.pose.orientation.w = q4

    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo(client.get_result())
    return True