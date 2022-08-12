#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Primitive properties

Contains spatial and temporal information logged during
navigation.
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

from urdfpy import URDF
from termcolor import colored
from gazebo_msgs.msg import ContactsState

from property_based_tester.temporal_cache.data_depot import data_reader
from property_based_tester.configuration.config import Configuration

class PrimitiveProperties():

    def __init__(self) -> None:

        self.config = Configuration()   

        self.in_collision = False
        self.robot_urdf_file = self.config.robots_dir+'/urdf/'+self.config.robot_urdf+'.urdf'
           
        rospy.Subscriber("bumper_contact_state", ContactsState, self.robot_force_sensor_callback)

    def physical_information(self):

        # https://readthedocs.org/projects/urdfpy/downloads/pdf/latest/
        
        robot = URDF.load(self.robot_urdf_file)

    def spatial_temporal_information(self):

        data = data_reader(self.config.workspace+'/src/property_based_tester/temporal_cache/logs/test')
        data = data.drop(columns=['Unnamed: 0'])

        self.robot_data = data.loc[data['Models'] == self.config.robot_urdf]

    def robot_force_sensor_callback(self, data):

        info = data.states
        if info:
            collider_1 = info[0].collision1_name
            collider_2 = info[0].collision2_name
            self.in_collision = True       

# a = PrimitiveProperties()
# a.physical_information()
# a.spatial_information()