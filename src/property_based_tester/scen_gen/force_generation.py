#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Obstacle Generator

Generates obstacles and gathers worlds properties.
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

from termcolor import colored
from gazebo_msgs.srv import ApplyBodyWrench 
from geometry_msgs.msg import Wrench



def apply_force(x=0,y=15,z=0,link='base_link',timeout=10,randomized=False): 
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

        force(body_name = link, wrench = wrench, duration = rospy.Duration(timeout))

    except:
            print(colored('Cannot apply force torque','red')) 