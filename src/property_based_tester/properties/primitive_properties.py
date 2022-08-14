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

from property_based_tester.temporal_cache.data_depot import data_reader
from property_based_tester.configuration.config import Configuration

class PrimitiveProperties():

    def __init__(self) -> None:

        self.config = Configuration()   
        self.robot_urdf_file = self.config.robots_dir+'/urdf/'+self.config.robot_urdf+'.urdf'
           
    def physical_information(self):

        # https://readthedocs.org/projects/urdfpy/downloads/pdf/latest/
        
        robot = URDF.load(self.robot_urdf_file)

    def spatial_temporal_information(self):
        ''' Extracts spatial information during the entire scenario. position is based on the 0,0 of gazebo.
        '''
        data = data_reader(self.config.workspace+'/src/property_based_tester/temporal_cache/logs/test')
        data = data.drop(columns=['Unnamed: 0'])

        robot_data = data.loc[data['Models'] == self.config.robot_urdf]\

        time_step = robot_data['Time'].to_numpy().T

        robo_pos_x = robot_data['X-pos'].to_numpy().astype(float)
        robo_pos_y = robot_data['Y-pos'].to_numpy().astype(float)
        robo_pos_z = robot_data['Z-pos'].to_numpy().astype(float)

        robo_pos_all = np.vstack((np.vstack((robo_pos_x,robo_pos_y)),robo_pos_z)).T

        return robo_pos_all

# a = PrimitiveProperties()
# a.physical_information()
# a.spatial_temporal_information()