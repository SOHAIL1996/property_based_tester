#!/usr/bin/env python
"""
Pick client: The client activates the pick action based on the MDR client.
"""
import rospy
import numpy as np
import actionlib

'''Depreciated Libraries'''
# from mdr_pickup_action.msg import PickupAction, PickupGoal

def picker_client(x, y, z, R,P,Y):
    """Iitialized pick action for the Toyota HSR using the MDR repository of MAS.

    Args:
        x (float): x-coordinates for pick action
        y (float): y-coordinates for pick action
        z (float): z-coordinates for pick action

        R (float): r-orientation for pick action
        P (float): p-orientation for pick action
        Y (float): y-orientation for pick action

    Returns:
        bool: Returns True if action commands were successful.
    """
    
    client = actionlib.SimpleActionClient('pickup_server', PickupAction)
    client.wait_for_server()

    goal = PickupGoal()
    goal.pose.header.frame_id = 'odom'
    goal.pose.header.stamp = rospy.Time.now()
    
    # goal.pose.pose.position.x = 1.518
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
    rospy.sleep(1)

    rospy.loginfo(client.get_result())
    return True