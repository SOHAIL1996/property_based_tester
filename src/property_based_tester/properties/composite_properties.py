#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Composite Properties

Culmination of composite properties that can used.
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
from gazebo_msgs.msg import ContactsState

from property_based_tester.configuration.config import Configuration
from property_based_tester.properties.primitive_properties import PrimitiveProperties

class CompositeProperties():

    def __init__(self) -> None:
        
        self.in_collision = False

        self.config = Configuration()
        self.primitive_properties = PrimitiveProperties()

        print(self.primitive_properties.spatial_information())

        rospy.Subscriber("bumper_contact_state", ContactsState, self.robot_collision_callback)
        
    def robot_collision_callback(self, data):
        info = data.states
        if info:
            collider_1 = info[0].collision1_name
            collider_2 = info[0].collision2_name
            self.in_collision = True         