#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Random Scenario Generator (RSG)

The RSG will be used to generate unique and altered
poses for the models that will be used within the
worlds of Gazebo simulator.
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
import numpy as np

import rospy

from geometry_msgs.msg import *
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest, DeleteModelRequest

from property_based_tester.configuration.config import bound_box
from property_based_tester.configuration.config import Configuration

from termcolor import colored


class RobotModel():
    """
    This class contains all relevant information of a gazebo model.
    """    
    def __init__(self, model_name,x=0,y=0,z=0,R=0,P=0,Y=0):
        
        conf = Configuration()

        self.model_dir       = conf.model_dir
        self.model_name      = model_name

        self.object_pose = Pose()
        self.object_pose.position.x = x
        self.object_pose.position.y = y
        self.object_pose.position.z = z
        self.object_pose.orientation.x = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
        self.object_pose.orientation.y = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
        self.object_pose.orientation.z = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
        self.object_pose.orientation.w = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)

        self.bounding_box = bound_box(self.model_real_name)
        
    def spawn_robot(self, model_type, model_name, file_location):
        """
        This function spawns the robot.
        
        Args:
            model_name (str): Name of the object and the object ID from which the model will be obtained.
        """
        model_desc = '{model_name: '+ model_name +'}'

        if model_type == "urdf":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            file_xml = open(file_location)
            xml_string=file_xml.read()

        elif model_type == "urdf.xacro":
            p = os.popen("rosrun xacro xacro.py " + file_location)
            xml_string = p.read()
            p.close()
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        elif model_type == "model":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
            file_xml = open(file_location)
            xml_string=file_xml.read()
        else:
            rospy.logerr('Model type not know. model_type = ' + model_type)


		# check if object is already spawned
        srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = self.model_name
        exists = True
        try:
            res = srv_delete_model(self.model_name)
        except rospy.ServiceException:
            exists = False
            rospy.logdebug("Model %s does not exist in gazebo.", self.model_name)

        if exists:
            rospy.loginfo("Model %s already exists in gazebo. Model will be updated.", self.model_name)

        # spawn new model
        req = SpawnModelRequest()
        req.model_name = self.model_name # model name from command line input
        req.model_xml = xml_string
        req.initial_pose = self.object_pose

        res = srv_spawn_model(req)

        # evaluate response
        if res.success == True:
            rospy.loginfo(res.status_message + " " + self.model_name)
        else:
            print("Error: Robot not spawned.")
		