#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Data Depot: Logs the temporal properties of the models.
"""
import numpy as np
import pandas as pd
import rospy
from datetime import datetime
from geometry_msgs.msg import *
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetWorldProperties

def world_state_extractor():
        """It extracts the information of all models in the gazebo world.

        Returns:
            list: Returns list of all relevant information
        """        
        now = datetime.now()
        # logs_objects = [['Models', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4','Time']]
        logs_objects = []

        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/get_link_state")

        try:
            world_service = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
            model_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

            world_props = world_service()
            
            for objects in world_props.model_names:
                coordinates = model_service(objects,'world')

                temp_model_data = [objects, # object name
                               np.around(coordinates.pose.position.x,3), # Position x
                               np.around(coordinates.pose.position.y,3), # Position y
                               np.around(coordinates.pose.position.z,3), # Position z
                               np.around(coordinates.pose.orientation.x,3), # Quaternion x
                               np.around(coordinates.pose.orientation.y,3), # Quaternion y
                               np.around(coordinates.pose.orientation.z,3), # Quaternion z
                               np.around(coordinates.pose.orientation.w,3), # Quaternion w
                               now.strftime("%H:%M:%S")] # time
                logs_objects.append(temp_model_data)    
            return logs_objects

        except rospy.ServiceException as e:
            print('Cannot acquire Model State.') 
        return None

def data_logger(df, loc):
    """Writes data into a csv file.

    Args:
        loc (str): file path.
    """   
    df.to_csv(loc +'_logs.csv', mode= 'w' ,encoding='utf-8')      
    
def data_reader(loc):
    """Reads .csv files and returns two data frames, one is 
    all the models and their positions.

    Args:
        loc (str): code name for calling the log file

    Returns:
        df_1 data_frame: Position and orientation of all the models.
    """    
    df_1 = pd.read_csv(loc +'_logs.csv')

    return df_1

def log_reader_comparator(loc, action_start, action_end):
    """Reads nav files for testing purposes.

    Args:
        loc (str): the column to be compared in testing
        
        action (str): the filename of the action test e.g. nav_start, nav_end
        
    Returns:
        int: Returns 2 numbers which are the difference in the log file,
             note the robot change in position in not taken into consideration.
    """    
    nswp, ignore = data_reader('logger/logs/'+action_start)
    newp, ignore = data_reader('logger/logs/'+action_end)
           
    nswp = nswp.replace('-0.000', '0.000')
    newp = newp.replace('-0.000', '0.000')
    
    newp = newp.set_index("Models")
    newp = newp.drop("hsrb", axis=0)
    nswp = nswp.set_index("Models")
    nswp = nswp.drop("hsrb", axis=0)

    # Checking if objects have moved -+0.05 cm or +-5 mm
    newp['8'] = np.where(nswp[loc]-0.05<=newp[loc], 1, 0) 
    newp['9'] = np.where(newp[loc]<=nswp[loc]+0.05, 1, 0)
             
    expected_difference_lower_tolerance = newp['8'].sum()
    original_difference_lower_tolerance = len(newp['8'])
    if expected_difference_lower_tolerance == original_difference_lower_tolerance:
        low = True
    else: 
        low = False
    
    expected_difference_upper_tolerance = newp['9'].sum()
    original_difference_upper_tolerance = len(newp['9'])
    
    if expected_difference_upper_tolerance == original_difference_upper_tolerance:
        up = True
    else: 
        up = False
    
    return low,up

def log_hsrb_reader():
    """Extracts the robots location

    Returns:
        list: returns the x y z quat1 quat2 quat3 quat4
    """    
    nav_start_world_props, ignore = data_reader('logger/logs/nav_start')
    nav_end_world_props, ignore = data_reader('logger/logs/nav_end')
    
    nav_end_world_props = nav_end_world_props.set_index("Models")
    hsrb = nav_end_world_props.loc["hsrb"]
    hsrb = hsrb.values.tolist()[1:] 
    return hsrb

def lucy_gripper_information():
    """Extracts the gripper information

    Returns:
        list: returns the x y z quat1 quat2 quat3 quat4
    """    
    ignore, gripper = data_reader('logger/logs/pick_action_end')
    gripper = gripper.set_index("Gripper Position")
    gripper_l = gripper.loc["Lucy left gripper"]
    gripper_r = gripper.loc["Lucy right gripper"]
    gripper_l = gripper_l.values.tolist()[1:]
    gripper_r = gripper_r.values.tolist()[1:]
    return gripper_l, gripper_r

def object_information(object_name, action):
    """Extracts an objects information.

    Returns:
        list: returns the x y z quat1 quat2 quat3 quat4
    """    
    end_action_model_states, ignore = data_reader('logger/logs/'+action)
    end_action_model_states = end_action_model_states.set_index("Models")
    object_properties = end_action_model_states.loc[object_name]
    object_properties = object_properties.values.tolist()[1:] 
    return object_properties