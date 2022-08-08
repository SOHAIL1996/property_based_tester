#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Picking Tests

Tests Lucy in a variety of parameterized and randomized
pick-action scenarios.
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
import rospy
import numpy as np
import pandas as pd
from subprocess import check_output
import pytest
from termcolor import colored
import actionlib

from mdr_pickup_action.msg import PickupAction, PickupGoal

from property_based_tester.utilities.Omni_base_locator.oml import OmniListener
from property_based_tester.scen_gen.obstacle_gen import Model
from property_based_tester.tests.file_reader.file_reader import Configuration
from property_based_tester.robot_controllers.pick_client import picker_client
from property_based_tester.robot_controllers.pick_Toyota_client import MoveItPickAndPlace
from hypothesis import given, settings, Verbosity, example
from property_based_tester.logger.data_logger import data_logger, lucy_gripper_information
from property_based_tester.logger.data_logger import data_reader, object_information
from property_based_tester.logger.data_logger import log_reader_comparator
from property_based_tester.logger.data_logger import log_hsrb_reader
import hypothesis.strategies as st

global destination_coord
destination_coord = []

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
                
@pytest.mark.usefixtures('set_up')         
class TestPickAction(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters

    def test_set_up(self,randomizer):
        """Initializing the test scenario.
        """  
        rospy.init_node('pick_test', anonymous=True)
        mo = Model('glass')   
        hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2]
        base_obj = Model(self.config.Platform_for_obstacle_pick, hx+0.8, hy, hz)
        base_obj.insert_model()
        pick_obj = Model(self.config.Obstacles_for_pick, hx+0.75, hy, 0.44)
        pick_obj.insert_model() 
        
    def test_verification_of_pick_action(self):
        """Activating the pick action test and checking whether it was successful.
        """
        mo = Model('glass')   
        data_logger('logger/logs/pick_action_start')
        hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2] 
        # result = picker_client(hx+0.65, hy, 0.5, 0.0, 0.0, 0.0)
        # Pick from table and drop on table
        # result = MoveItPickAndPlace( pick_x = hx+0.75, pick_y = hy, pick_z = 0.55, 
        #                              place_x = hx+0.7, place_y = hy, place_z = 0.76)
        # Pick from table and place on shelf
        result = MoveItPickAndPlace(pick_x = hx+0.75, pick_y = hy+0.05, pick_z = 0.55, 
                                     place_x = 2.933, place_y = 3.107 , place_z = 0.46)
        data_logger('logger/logs/pick_action_end')
        
    def test_object_proximity_verification(self):
        """Verification if objects are ahead of Lucy.
        """  
        log, lucy_log = data_reader('logger/logs/pick_action_end')
        log = log.set_index("Models")
        hsrb = log.loc["hsrb"]
        hsrb = hsrb.values.tolist()[1:] 
        log = log.drop("hsrb", axis=0)
        log = log.drop("ground_plane", axis=0)
        log = log.drop(self.config.World, axis=0, errors='ignore')

        x = log['X-pos'].values.tolist()
        x_res = any(hsrb[0]-0.5 <= ele <= hsrb[0]+0.5 for ele in x)

        y = log['Y-pos'].values.tolist()
        y_res = any(hsrb[1]-0.5 <= ele <= hsrb[1]+0.5 for ele in y)

        assert True == x_res and y_res  
    
    def test_collision_detection(self):
        """ Checking if the position of objects changed during pick action i.e. Lucy collided with an obstacle.
        """    
        # True means no change and false means there is change.
        x = False
        y = False
        z = False
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('X-pos', 'pick_action_start', 'pick_action_end')
        if lower_tolerance_difference == upper_tolerance_difference:
            x = True
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Y-pos', 'pick_action_start', 'pick_action_end')
        if lower_tolerance_difference == upper_tolerance_difference:
            y = True
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Z-pos', 'pick_action_start', 'pick_action_end')
        if lower_tolerance_difference == upper_tolerance_difference:
            z = True
        assert (x or y or z) == False

    def test_object_final_position(self):
        """ Checking if the designated obstacle is between lucys grippers.
        """    
        object_properties = object_information(self.config.Obstacles_for_pick, 'pick_action_end')
        lucy_gripper_l, lucy_gripper_r = lucy_gripper_information()
        assert lucy_gripper_l[0]-0.15 <= object_properties[0] <= lucy_gripper_r[0]+0.15
        assert lucy_gripper_l[1]-0.15 <= object_properties[1] <= lucy_gripper_r[1]+0.15
    
    def test_operation_zone(self):
        """ Checking if the projected position of the robot has not gone out of the boundary. 
        The boundary is the test lab navigation map and its dimensions are 10x10 m^2.
        """    
        hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2]
        omni = OmniListener()
        omni.omnibase_listener()
        x,y,z = omni.x, omni.y, omni.z
        assert -5 <= hx <= 5
        assert -5 <= hy <= 5
        assert -5 <= x <= 5
        assert -5 <= y <= 5 
        
    def test_tear_down(self):
        """Tearing down the setup for the pick test.
        """  
        # test = Model('glass')   
        # test.delete_model(self.config.Obstacles_for_pick)
        # test.delete_model(self.config.Platform_for_obstacle_pick)
        # Attaching log file to the test results
        logs = self.config.config_data_frame('pick_action_end')
        data = logs.to_csv(index=False)
        allure.attach(data, 'Configuration', allure.attachment_type.CSV)