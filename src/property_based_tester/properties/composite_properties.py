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

from property_based_tester.configuration.config import Configuration
from property_based_tester.properties.primitive_properties import PrimitiveProperties

from gazebo_msgs.msg import ContactsState

class CompositeProperties():

    def __init__(self) -> None:
        
        self.config = Configuration()
        self.primitive_properties = PrimitiveProperties()

        self.in_collision = False 
        rospy.Subscriber("bumper_contact_state", ContactsState, self.robot_force_sensor_callback)

    def must_be_at(self, robot, area, time, tolerance):
        pass

    def must_not_be_at(self, robot, area, time, tolerance):
        pass

    def must_be_near_to(self, robot, object, time, tolerance):
        pass

    def must_not_be_near_to(self, robot, object, time, tolerance):
        pass

    def must_collide(self, robot, object):
        pass

    def must_not_collide(self, robot, object):
        pass

    def robot_force_sensor_callback(self, data):
        info = data.states
        if info:
            collider_1 = info[0].collision1_name
            collider_2 = info[0].collision2_name
            self.in_collision = True       