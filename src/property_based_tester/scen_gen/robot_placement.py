#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Robot Placement: Spawns the mobile robot/robotic arm in the 
Gazebo simulator. 
"""
import os
import numpy as np

import rospy

from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest, DeleteModelRequest, SetModelState

from property_based_tester.configuration.config import bound_box
from property_based_tester.configuration.config import Configuration

from termcolor import colored


class RobotModel():
    """
    This class contains all relevant information of a gazebo model.
    """    
    def __init__(self, model_name,x=0,y=0,z=0,R=0,P=0,Y=0):
        
        conf = Configuration()

        self.robo_dir        = conf.robots_dir+'/urdf/'+model_name+'.urdf'
        self.model_name      = model_name

        R, P, Y = R*((22/7)/180), P*((22/7)/180), Y*((22/7)/180)

        self.object_pose = Pose()
        self.object_pose.position.x = x
        self.object_pose.position.y = y
        self.object_pose.position.z = z
        self.object_pose.orientation.x = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2) 
        self.object_pose.orientation.y = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2) 
        self.object_pose.orientation.z = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2) 
        self.object_pose.orientation.w = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2) 

        # self.bounding_box = bound_box(self.model_real_name)
        
    def spawn_robot(self, model_type):
        """
        This function spawns the robot.
        
        Args:
            model_name (str): Name of the object and the object ID from which the model will be obtained.
        """


        if model_type == "urdf":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            file_xml = open(self.robo_dir)
            xml_string=file_xml.read()

        elif model_type == "urdf.xacro":
            p = os.popen("rosrun xacro xacro.py " + self.robo_dir)
            xml_string = p.read()
            p.close()
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        elif model_type == "model":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
            file_xml = open(self.robo_dir )
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

    def robot_pose(self):

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
           self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException :
            print("Service call failed")

        state_msg = ModelState()
        state_msg.model_name =  self.model_name

        state_msg.pose.position.x = self.object_pose.position.x
        state_msg.pose.position.y = self.object_pose.position.y
        state_msg.pose.position.z = self.object_pose.position.z
        state_msg.pose.orientation.x = self.object_pose.orientation.x
        state_msg.pose.orientation.y = self.object_pose.orientation.y
        state_msg.pose.orientation.z = self.object_pose.orientation.z
        state_msg.pose.orientation.w = self.object_pose.orientation.w

        resp = self.set_state(state_msg)