#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Obstacle Generator: Generates obstacles and gathers worlds properties.
"""

import rospy

from termcolor import colored
from gazebo_msgs.srv import ApplyBodyWrench 
from geometry_msgs.msg import Wrench

def apply_force(x=0,y=0,z=0,link='base_link',timeout=10,randomized=False): 
    """Applies a randomized force to a target entity in the Gazebo simulator.

    Args:
        x (int, optional): The force to be applied in the x-axis. Defaults to 0.
        y (int, optional): The force to be applied in the y-axis. Defaults to 15.
        z (int, optional): The force to be applied in the z-axis. Defaults to 0.
        link (str, optional): The target link for applying the force. Defaults to 'base_link'.
        timeout (int, optional): The amount of time to apply the force. Defaults to 10.
        randomized (bool, optional): Whether the force should be randomized. Defaults to False.
    """
    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench') 
        force = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)

        wrench          = Wrench()
        wrench.force.x  = x
        wrench.force.y  = y
        wrench.force.z  = z
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

        force(body_name=link, wrench = wrench, duration = rospy.Duration(timeout))

    except:
            print(colored('Cannot apply force torque','red')) 