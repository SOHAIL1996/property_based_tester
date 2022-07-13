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
import time
import random
import subprocess
import numpy as np

import rospy
import std_srvs.srv as std_srvs
import gazebo_msgs.srv as gazebo_srvs
from geometry_msgs.msg import *
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetWorldProperties

from property_based_tester.configuration.config import bound_box
from property_based_tester.configuration.config import Configuration
from property_based_tester.configuration.config import choices

from termcolor import colored

global model_tracer
   
class World():
    """
    This class contains all relevant information of a gazebo world.
    """    
    def __init__(self):
        pass
    
    def world_properties(self):
        try:
            print(colored('Getting world properties.','yellow'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/get_world_properties'])
        finally:
            time.sleep(2)
            world_node.terminate()     
    
    def reset_world(self):
        """
        This class contains resets the models poses.
        """    
        try:
            print(colored('Resetting model poses.','yellow'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/reset_world'])
        finally:
            time.sleep(2)
            world_node.terminate() 
            
    def reset_simulation(self):
        """
        This class contains resets everything in gazebo.
        """    
        try:
            print(colored('Resetting everything.','yellow'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/reset_simulation'])
        finally:
            time.sleep(2)
            world_node.terminate()  
            
    def unpause_simulation(self):
        """
        This class un pauses the physics engine in gazebo.
        """    
        try:
            print(colored('Un-pausing physics engine.','green'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/unpause_physics'])
        finally:
            time.sleep(2)
            world_node.terminate() 
            
    def world_state(self):
        """It extracts the information of all models in the gazebo world.

        Returns:
            [list]: Returns list of all relevant information
        """        
        logs = [['Models', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4']]
        lucy_logs = [['Gripper Position', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4']]
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/get_link_state")
        try:
            # print(colored('Acquiring Model State','yellow'))
            
            world = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
            model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            end_of_effector_left = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            end_of_effector_right= rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

            world_props = world()
            lucy_lef_gripper = end_of_effector_left('hsrb::hand_l_distal_link','world')
            lucy_rig_gripper = end_of_effector_right('hsrb::hand_r_distal_link','world')
            for objs in world_props.model_names:
                coordinates = model(objs,'world')
                temp_data   = [objs,np.around(coordinates.pose.position.x,3),
                               np.around(coordinates.pose.position.y,3),
                               np.around(coordinates.pose.position.z,3),
                               np.around(coordinates.pose.orientation.x,3),
                               np.around(coordinates.pose.orientation.y,3),
                               np.around(coordinates.pose.orientation.z,3),
                               np.around(coordinates.pose.orientation.w,3)]
                logs.append(temp_data)    
                

            lucy_data_lef  = ['Lucy left gripper',np.around(lucy_lef_gripper.link_state.pose.position.x,3),
                            np.around(lucy_lef_gripper.link_state.pose.position.y,3),
                            np.around(lucy_lef_gripper.link_state.pose.position.z,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.x,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.y,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.z,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.w,3)]
            
            lucy_data_rig  = ['Lucy right gripper',np.around(lucy_rig_gripper.link_state.pose.position.x,3),
                            np.around(lucy_rig_gripper.link_state.pose.position.y,3),
                            np.around(lucy_rig_gripper.link_state.pose.position.z,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.x,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.y,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.z,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.w,3)]
            
            lucy_logs.append(lucy_data_lef)
            lucy_logs.append(lucy_data_rig) 
            
            # print(colored('Successfully acquired Model State','green')) 
            return logs, lucy_logs
        except rospy.ServiceException as e:
            print(colored('Cannot acquire Model State.','red')) 
        return 
               
class Model():
    """
    This class contains all relevant information of a gazebo model.
    """    
    def __init__(self, model_real_name,x=0,y=0,z=0,R=0,P=0,Y=0):
        
        conf = Configuration()

        self.model_dir       = conf.model_dir
        self.model_real_name = model_real_name
        self.model_node_name = str(np.random.randint(0,999)) 
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
        
        self.bounding_box = bound_box(self.model_real_name)
        
    def model_properties(self, model_name):
        """
        This function obtains a models property.
        
        Args:
            model_name (str): Name of the object and the object ID from which the model will be obtained.
        """
        model_desc = '{model_name: '+ model_name +'}'

        try:
            print(colored('Getting {0} model properties.'.format(model_name),'yellow'))
            property_node = subprocess.Popen(['rosservice', 'call', 'gazebo/get_model_properties', model_desc])
        finally:
            time.sleep(1.5)
            property_node.terminate()   
            print(colored('Successfully obtained {0} model properties.'.format(model_name),'green')) 
               
    def insert_model(self):
        """
        This function inserts a model in the gazebo world by passing a roscommand from the terminal. 
        """        
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        conf = Configuration()
        try:
            print(colored('Inserting {0} model'.format(self.model_real_name),'yellow'))
            
            # Un-comment if first time using the package. Corrects the uri of sdf models.
            # conf.model_uri_correcter(self.model_real_name)
            
            model_node = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            if self.model_real_name == conf.world:
                model_node(model_name= self.model_real_name, 
                       model_xml = open(self.model_dir +'/'+self.model_real_name+'/'+self.model_real_name+'.sdf', 'r').read(), 
                       robot_namespace = "/RSG",
                       initial_pose = Pose(position=Point(self.x_coord,self.y_coord,self.z_coord),
                                          orientation=Quaternion(self.quat1,self.quat2,self.quat3,self.quat4)),
                       reference_frame ="world")
            else:
                model_node(model_name= self.model_real_name + self.model_node_name, 
                        model_xml = open(self.model_dir +'/'+self.model_real_name+'/'+self.model_real_name+'.sdf', 'r').read(), 
                        robot_namespace = "/RSG",
                        initial_pose = Pose(position=Point(self.x_coord,self.y_coord,self.z_coord),
                                            orientation=Quaternion(self.quat1,self.quat2,self.quat3,self.quat4)),
                        reference_frame ="world")
            print(colored('Successfully spawned {0} model'.format(self.model_real_name),'green'))
            
        except rospy.ServiceException as e:
            print(colored('Cannot spawn {0} model'.format(self.model_real_name),'red')) 

    def delete_model(self, model_name):
        """
        This function deletes a model in the gazebo world by passing a roscommand from the terminal.
        
        Args:
            model_name (str): Name of the object and the object ID to be deleted.
        """
        model_desc = '{model_name: '+ model_name +'}'

        try:
            print(colored('Deleting {0} model'.format(model_name),'yellow'))
            delete_node = subprocess.Popen(['rosservice', 'call', 'gazebo/delete_model', model_desc])
        finally:
            time.sleep(2)
            delete_node.terminate()   
            print(colored('Successfully deleted {0} model'.format(model_name),'green'))   