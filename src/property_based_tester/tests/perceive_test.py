#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Perceive Test module

Tests Lucy in a variety of parameterized and randomized
perception scenarios.
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
import allure

from gazebo_msgs.srv import GetLightProperties
from mdr_pickup_action.msg import PickupAction, PickupGoal

from property_based_tester.tests.obstacle_generator.obstacle_gen import Model
from property_based_tester.tests.file_reader.file_reader import Configuration
from property_based_tester.tests.world_properties.world_prop import world_state
from property_based_tester.tests.action_client.perceive_client import perceive_client

from hypothesis import given, settings, Verbosity, example

from property_based_tester.logger.data_logger import data_logger
from property_based_tester.logger.data_logger import data_reader
from property_based_tester.logger.data_logger import log_reader_comparator
from property_based_tester.logger.data_logger import log_hsrb_reader

import hypothesis.strategies as st

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
        
@pytest.mark.usefixtures('set_up')         
class TestPerception(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters
    
    # @allure.severity('minor')   
    def test_set_up(self,randomizer):
        """Initialzing parameters for testing.
        """        
        self.config = Configuration() 
        rospy.init_node('perception_node') 
        # Obtaining Lucy's position
        lucy_loc = Model('glass')   
        hx,hy,hz = lucy_loc.lucy_pos()[0],lucy_loc.lucy_pos()[1],lucy_loc.lucy_pos()[2]
        # Loading in a objects for perception
        base_obj = Model(self.config.Platform_for_obstacle_perc, hx+0.8, hy, hz)
        base_obj.insert_model()
        perceive_obj = Model(self.config.Obstacles_for_perc, hx+0.6, hy, 0.72)
        perceive_obj.insert_model() 

    def test_verification_of_perception(self):
        """Activates perception action.
        """
        data_logger('logger/logs/perception')
        result = perceive_client()    
        assert True == result
    
    def test_object_proximity_verification(self):
        """Verification if object is ahead.
        """  
        log, lucy_log = data_reader('logger/logs/perception')
        log = log.set_index("Models")
        hsrb = log.loc["hsrb"]
        hsrb = hsrb.values.tolist()[1:] 
        log = log.drop("hsrb", axis=0)
        log = log.drop("ground_plane", axis=0)
        log = log.drop(self.config.World, axis=0)

        x = log['X-pos'].values.tolist()
        x_res = any(hsrb[0]-0.5 <= ele <= hsrb[0]+0.5 for ele in x)

        y = log['Y-pos'].values.tolist()
        y_res = any(hsrb[1]-0.5 <= ele <= hsrb[1]+0.5 for ele in y)

        assert True == x_res and y_res    
        
    def test_lighting_conditions(self):
        """Verification of illumination levels.
        """
        rospy.wait_for_service('/gazebo/get_light_properties')
        try:
            light_prop = rospy.ServiceProxy('/gazebo/get_light_properties',GetLightProperties)
            light_props = light_prop('sun')
            color_spectrum = light_props.diffuse
        except rospy.ServiceException as e:
            print(colored('Cannot acquire ligh State.','red')) 
        assert (color_spectrum.r or color_spectrum.g or color_spectrum.b) > 0.5        
        
    def test_tear_down(self):
        """Obstacle removal.
        """  
        # Deleting spawned models
        test = Model('glass')  
        test.delete_model(self.config.Obstacles_for_perc)
        test.delete_model(self.config.Platform_for_obstacle_perc)
        # Attaching log file to the test results
        logs = self.config.config_data_frame('perception')
        data = logs.to_csv(index=False)
        allure.attach(data, 'Configuration', allure.attachment_type.CSV)