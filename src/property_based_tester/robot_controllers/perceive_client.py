#! /usr/bin/env python
"""
Perceive client: The client activates the perceive action based
on the updated MDR module.
"""
import rospy
import actionlib

'''Depreciated Libraries'''
# from mdr_perceive_plane_action.msg import PerceivePlaneAction, PerceivePlaneGoal 
# from hsrb_interface import Robot 
    
def perceive_client():
    """Initializes and runs the Toyota Human Support Robots Perception client.

    Returns:
        bool: Returns True if the client successfully started.
    """

    robot = Robot()
    whole_body = robot.get('whole_body')
    whole_body.move_to_joint_positions({'head_tilt_joint': -0.3})
    
    client = actionlib.SimpleActionClient('/mdr_actions/perceive_plane_server', PerceivePlaneAction)
    client.wait_for_server()
    goal = PerceivePlaneGoal()
    goal.plane_config = 'table'
    goal.plane_frame_prefix = 'frame_table'
    try:
        timeout = 45.0
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    except:
        return False  
    return True