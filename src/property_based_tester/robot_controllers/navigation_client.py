#!/usr/bin/env python
"""
---------------------------------------------------- 
Navigation client

The client activates the MDR navigation module.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""
import numpy as np
import rospy
import actionlib
import time
import math
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def pose_action_client(coord_x, coord_y, direction):
    """Action client test for navigation using coordinates.

    Args:
        coord_x (float 32): x coordinate in the map for the robot to move towards.
        coord_y (float 32): y coordinate in the map for the robot to move towards.
        direction (float 32)(Degrees): Direction the robot should face.

    Returns:
        bool: Returns boolean rarely works in a laptop.
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    distance_tolerance = 0.5
    duration = 1

    # Change to map
    odom_frame_id = 'odom'
    base_frame_id = 'base_link'

    R = 0
    P = 0
    Y = np.deg2rad(direction)
        
    client.wait_for_server()
    goal = MoveBaseGoal()    
    listener = tf.TransformListener()
    try:
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = coord_x
        goal.target_pose.pose.position.y = coord_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
        goal.target_pose.pose.orientation.y = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
        goal.target_pose.pose.orientation.z = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
        goal.target_pose.pose.orientation.w = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)

        client.send_goal(goal)

        if not distance_tolerance > 0.0:
                client.wait_for_result()
                time.sleep(duration)
        else:
            distance = 10
            while(distance > distance_tolerance):
                try:
                    trans,rot = listener.lookupTransform(odom_frame_id,base_frame_id, rospy.Time.now())
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                distance = math.sqrt(pow(coord_x-trans[0],2)+pow(coord_y-trans[1],2))
            client.cancel_all_goals()
        return True

    except Exception as exc:
        rospy.logerr(exc)
        return False