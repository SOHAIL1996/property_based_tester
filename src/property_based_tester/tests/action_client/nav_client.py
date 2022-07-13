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
import std_msgs.msg
import hsrb_interface

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal

def navi_action_client(location):
    """Action client test for navigation using map.

    Args:
        loc (str): The location it has to go, updated in
        mas_domestic_robotics > mdr_environments > navigation goals.yaml

    Returns:
        bool: Returns boolean rarely works in my laptop.
    """    
    rospy.init_node('mdr_move_base_client_test')
    client = actionlib.SimpleActionClient('move_base_server', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()

    try:
        goal.goal_type = MoveBaseGoal.NAMED_TARGET
        goal.destination_location = location
        timeout = 120.0
        rospy.loginfo('Sending action lib goal to move_base_server, ' +
                    'destination : ' + goal.destination_location)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        print(client.get_result())
    except Exception as exc:
        print(str(exc))
        return False
    return True


def pose_action_client(coord_x, coord_y, direction):
    """Action client test for navigation using coordinates.

    Args:
        coord_x (float 32): x coordinate in the map for the robot to move towards.
        coord_y (float 32): y coordinate in the map for the robot to move towards.
        direction (float 32)(Degrees): Direction the robot should face.

    Returns:
        bool: Returns boolean rarely works in a laptop.
    """
    # rospy.init_node('mdr_move_base_client_test')
    client = actionlib.SimpleActionClient('move_base_server', MoveBaseAction)
    rospy.sleep(1)
    head = std_msgs.msg.Header(frame_id='map',stamp=rospy.Time.now())
    location = Point(x=coord_x, y=coord_y, z=0)
    R = 0
    P = 0
    Y = np.deg2rad(direction)
    quat1     = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
    quat2     = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
    quat3     = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
    quat4     = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)
    angle = Quaternion(x=quat1,y=quat2,z=quat3,w=quat4)
        
    client.wait_for_server()
    goal = MoveBaseGoal()    
    try:
        goal.goal_type = MoveBaseGoal.POSE    
        goal.pose = PoseStamped(header=head, pose=Pose(position=location, orientation=angle))       
        timeout = 1000.0
        rospy.loginfo('Sending action lib goal to move_base_server, ' + goal.destination_location)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        rospy.sleep(3)
    except Exception as exc:
        print(str(exc))
        return False
    return True

def toyota_action_client(coord_x, coord_y, direction):
    """Action client test for navigation using coordinates.

    Args:
        coord_x (float 32): x coordinate in the map for the robot to move towards.
        coord_y (float 32): y coordinate in the map for the robot to move towards.
        direction (float 32)(Degrees): Direction the robot should face.

    Returns:
        bool: Returns boolean rarely works in a laptop.
    """
    robot = hsrb_interface.Robot()
    omni_base = robot.get('omni_base')
    with hsrb_interface.Robot() as robot:
        base = robot.try_get('omni_base')
        base.go_abs(coord_x, coord_y, direction)
        
    return True
           