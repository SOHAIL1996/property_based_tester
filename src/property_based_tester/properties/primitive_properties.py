#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Primitive properties: Processes spatial and temporal information logged during
scenario execution.
"""

import rospy
import numpy as np

from urdfpy import URDF
from termcolor import colored
from scipy.spatial.transform import Rotation

from property_based_tester.temporal_cache.data_depot import data_reader
from property_based_tester.configuration.config import Configuration

class PrimitiveProperties():

    def __init__(self) -> None:

        self.config = Configuration()   
        self.robot_urdf_file = self.config.robots_dir+'/urdf/'+self.config.robot_urdf+'.urdf'
           
    def physical_information(self):
        """Extracts the physical information of the robot such as inertia from the urdf.
        """

        # https://readthedocs.org/projects/urdfpy/downloads/pdf/latest/
        robot = URDF.load(self.robot_urdf_file)

    def robo_spatial_temporal_information(self, object):
        """Extracts spatial information of the robotduring the entire scenario. Position is based on the 0,0 world frame of gazebo.

        Args:
            object (str): Robot name for information extraction.

        Returns:
            Numpy Matrix: All information of the logged data during scenario exectuion.
        """
        data = data_reader(self.config.workspace+'/src/property_based_tester/temporal_cache/logs/test')
        data = data.drop(columns=['Unnamed: 0'])

        robot_data = data.loc[data['Models'] == object]

        time_step = robot_data['Time'].to_numpy().T

        robo_pos_x = robot_data['X-pos'].to_numpy().astype(float)
        robo_pos_y = robot_data['Y-pos'].to_numpy().astype(float)
        robo_pos_z = robot_data['Z-pos'].to_numpy().astype(float)

        robo_pos_all = np.vstack((np.vstack((robo_pos_x,robo_pos_y)),robo_pos_z)).T

        return robo_pos_all
    
    def spatial_temporal_information(self, object):
        """Extracts spatial information during the entire scenario. Position is based on the 0,0 world frame of gazebo.

        Args:
            object (str): Target entity for information extraction.

        Returns:
            Numpy Matrix: All information of the logged data during scenario exectuion.
        """
        data = data_reader(self.config.workspace+'/src/property_based_tester/temporal_cache/logs/test')
        data = data.drop(columns=['Unnamed: 0'])

        object_data = data.loc[data['Models'] == object]

        time_step = object_data['Time'].to_numpy().T

        object_pos_q1 = object_data['Q-1'].to_numpy().astype(float)
        object_pos_q2 = object_data['Q-2'].to_numpy().astype(float)
        object_pos_q3 = object_data['Q-3'].to_numpy().astype(float)
        object_pos_q4 = object_data['Q-4'].to_numpy().astype(float)

        all_orientations = np.zeros([1, 3])

        for i in range(len(object_pos_q1)):
            r,p,y = self.quaternion_to_euler_angle_vectorized2(object_pos_q4[i],object_pos_q1[i],object_pos_q2[i],object_pos_q3[i])
            all_orientations = np.vstack((all_orientations, [abs(r),abs(p),abs(y)]))

        return all_orientations

    def quaternion_to_euler_angle_vectorized2(self, w, x, y, z):
        """Converts Quaternions to Euler angles.

        Args:
            w (float): Quaternion 1
            x (float): Quaternion 2
            y (float): Quaternion 3
            z (float): Quaternion 4

        Returns:
            float: Roll, Pitch, Yaw
        """
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)

        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return np.round(X,2), np.round(Y,2), np.round(Z,2)

# a = PrimitiveProperties()
# a.physical_information()
# a.spatial_temporal_information('jackal_robot_issac')