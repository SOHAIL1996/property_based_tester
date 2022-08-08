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
import os
import time
import random
import subprocess
import numpy as np

import rospy
import std_srvs.srv as std_srvs
import gazebo_msgs.srv as gazebo_srvs
from geometry_msgs.msg import *

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel 
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetWorldProperties

from property_based_tester.configuration.config import Configuration
from termcolor import colored

class Model():
    """
    This class contains all relevant information of a gazebo model.
    """    
    def __init__(self, model_real_name, x=0, y=0, z=0, R=0, P=0, Y=0):
        
        conf = Configuration()

        self.model_dir       = conf.model_dir
        self.model_real_name = model_real_name
        self.model_number = ''
        self.x_coord   = x  # X-coordinate
        self.y_coord   = y  # Y-coordinate
        self.z_coord   = z  # Z-coordinate
        self.R_coord   = R  # Roll in radians
        self.P_coord   = P  # Pitch in radians
        self.Y_coord   = Y  # Yaw in radians
        self.quat1     = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
        self.quat2     = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
        self.quat3     = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
        self.quat4     = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)
        
    def model_properties(self, model_name):
        """
        This function obtains a models property.
        
        Args:
            model_name (str): Name of the object and the object ID from which the model will be obtained.
        """
        model_desc = '{model_name: '+ model_name +'}'

        try:
            property_node = subprocess.Popen(['rosservice', 'call', 'gazebo/get_model_properties', model_desc])
        finally:
            time.sleep(1.5)
            property_node.terminate()    
               
    def insert_model(self):
        """
        This function inserts a model in the gazebo world by passing a roscommand from the terminal. 
        """        
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        conf = Configuration()
        try:           
            model_node = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            model_node(model_name= self.model_real_name+self.model_number, 
                       model_xml = open(self.model_dir +'/'+self.model_real_name+'/'+self.model_real_name+'.sdf', 'r').read(), 
                       robot_namespace = "/RSG",
                       initial_pose = Pose(position=Point(self.x_coord,self.y_coord,self.z_coord),
                                          orientation=Quaternion(self.quat1,self.quat2,self.quat3,self.quat4)),
                       reference_frame ="world")
        except rospy.ServiceException as e:
            print(colored('Cannot spawn {0} model'.format(self.model_real_name),'red')) 

    def delete_model(self, model_name):
        """
        This function deletes a model in the gazebo world by passing a roscommand from the terminal.
        
        Args:
            model_name (str): Name of the object and the object ID to be deleted.
        """
        model_desc = '{model_name: '+ model_name +'}'
        rospy.wait_for_service("/gazebo/delete_model")
        try:
            del_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel) 
            del_model_prox(model_name) 
        except rospy.ServiceException as e:
            print(colored('\n Rospy.ServiceException \n','red')) 
            
    def lucy_pos(self):
        try:
            model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            coordinates = model('hsrb','world')
            log   = [np.around(coordinates.pose.position.x,3),
                            np.around(coordinates.pose.position.y,3),
                            np.around(coordinates.pose.position.z,3),
                            np.around(coordinates.pose.orientation.x,3),
                            np.around(coordinates.pose.orientation.y,3),
                            np.around(coordinates.pose.orientation.z,3),
                            np.around(coordinates.pose.orientation.w,3)] 
            return log
        except rospy.ServiceException as e:
            print(colored('Cannot acquire Model State.','red')) 