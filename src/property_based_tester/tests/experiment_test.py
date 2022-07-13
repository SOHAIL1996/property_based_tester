#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Experimental-Complex-scenarios 

Tests Lucy in a complex-scenario
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

import hsrb_interface

# from utilities.utils import *
from property_based_tester.tests.action_client.nav_client import navi_action_client
from property_based_tester.tests.action_client.nav_client import pose_action_client
from property_based_tester.utilities.Omni_base_locator.oml import OmniListener
from property_based_tester.tests.obstacle_generator.obstacle_gen import Model
from property_based_tester.tests.file_reader.file_reader import Configuration
from property_based_tester.logger.data_logger import log_reader_comparator
from property_based_tester.logger.data_logger import log_hsrb_reader
from hsrb_interface import Robot
from hypothesis import given, settings, Verbosity, example
from mdr_pickup_action.msg import PickupAction, PickupGoal
from tests.world_properties.world_prop import world_state
from tests.action_client.perceive_client import perceive_client
from tests.action_client.pick_place_client import MoveItPickAndPlace
from logger.data_logger import data_logger, lucy_gripper_information
from logger.data_logger import data_reader, object_information

global destination_coord
destination_coord = []

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
        
@pytest.mark.usefixtures('set_up')         
class TestComplexScenario(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters
    
    def test_set_up(self,randomizer):
        """Initializing navigation scenario.
        """  
        rospy.init_node('complex_scenario_test')
        
    def test_movement(self):
        robot = hsrb_interface.Robot()
        omni_base = robot.get('omni_base')
        with hsrb_interface.Robot() as robot:
           base = robot.try_get('omni_base')
           base.go_abs(0, 2.683, 0.0)
