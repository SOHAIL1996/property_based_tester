#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Navigation Tests

Tests a UGV in a variety of parameterized and randomized
navigation scenarios.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""

import random
import rospy
import numpy as np
import os, subprocess, signal, time
import pytest
import allure

from property_based_tester.configuration.config import Configuration
from property_based_tester.custom_tests.user_test_builder import UserTesting
from property_based_tester.property_based_language_generation.textx_test_specification import PropertyBasedLanguageGenerator
from property_based_tester.robot_controllers.navigation_client import pose_action_client
from property_based_tester.scen_gen.obstacle_gen import Model
from property_based_tester.scen_gen.model_placement import delete_model
from property_based_tester.scen_gen.robot_placement import RobotModel

from property_based_tester.properties.composite_properties import CompositeProperties


from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

global spawned_items
spawned_items = []

pblg = PropertyBasedLanguageGenerator()
pblg_config = pblg.scenario_composite_tests 

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
        self.composite_properties = CompositeProperties()
        self.user_tests = UserTesting()

@pytest.mark.usefixtures('set_up')    
@pytest.mark.parametrize('pblg_config', pblg_config, scope="class")
class TestScenario(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters

    @pytest.fixture()
    def robo_pose_correction(self):
        robo = RobotModel(self.config.robot_urdf,x=0,y=0,z=0.12,R=0,P=0,Y=0)
        robo.robot_pose()

    def pytest_configure():
        pytest.collision = False

    def test_set_up(self, robo_pose_correction, pblg_config):
        """Initializing property-based generator language scenario.
        """  
        rospy.init_node('pblg_test')

    def test_scenario_generation(self, randomizer, pblg_config):
        world = Model(pblg_config[0].world_type,0,0,0)
        world.insert_model()
        assert True
    
    # @settings(max_examples=1)
    # @given(st.sampled_from(['table','shelf','cabinet','sofa']))
    # def test_scenario_generation_map(destination): 
    #     """Defines a scenario for the rest of the tests to run in using navigation map.
    #     """    
    #     data_logger('logger/logs/nav_start')
    #     result = navi_action_client(destination)
    #     data_logger('logger/logs/nav_end')
    #     assert result == True

    # def test_scenario_execution(self, randomizer, pblg_config): 
    #     """Defines a scenario for the rest of the tests to run in using coodrinates.
    #     """    
    #     # Randomize given coordinates
    #     coord_x, coord_y, direction = randomizer(-2,2),randomizer(-2,2),randomizer(0,360)

    #     # Excute navigation and temporal logger
    #     temporal_logger = subprocess.Popen(['rosrun', self.config.rospkg_name, 'temporal_log.py'], preexec_fn=os.setsid)
    #     result = pose_action_client(coord_x, coord_y, direction)
    #     os.killpg(os.getpgid(temporal_logger.pid), signal.SIGTERM) 
    #     pytest.collision = self.composite_properties.in_collision

    #     assert result == True    

    # def test_must_collide(self, pblg_config):
    #     """ Checking if the robot collided with a specific obstacle during navigation.
    #     """    
    #     if self.user_tests.must_collide:
    #         assert pytest.collision == True
    #     else:
    #         pytest.skip("Uninitialized by user")

    # def test_must_not_collide(self, pblg_config):
    #     """ Checking if the robot has not collided with an obstacle during navigation.
    #     """    
    #     if self.user_tests.must_not_collide:
    #         assert pytest.collision == False
    #     else:
    #         pytest.skip("Uninitialized by user")

    def test_must_be_at(self, pblg_config):
        """ Checking if the robot is within a given area.
        """    
        check = False
        for configuration in pblg_config[1]:
            if configuration[0] == 'must_be_at':
                check = True

                assert self.composite_properties.must_be_at(target_area_min=[configuration[1].x1,configuration[1].y1,configuration[1].z1], 
                                                            target_area_max=[configuration[1].x2,configuration[1].y2,configuration[1].z2]) == True
                
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_not_be_at(self, pblg_config):
        """ Checking if the robot is not within a given area.
        """    
        check = False
        for configuration in pblg_config[1]:
            if configuration[0] == 'must_not_be_at':
                check = True

                assert self.composite_properties.must_be_at(target_area_min=[configuration[1].x1,configuration[1].y1,configuration[1].z1], 
                                                            target_area_max=[configuration[1].x2,configuration[1].y2,configuration[1].z2]) == True
                
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_static_obstacle_generation_tear_down(self, pblg_config):
        """Tearing down the setup for navigation.
        """  
        delete_model(pblg_config[0].world_type) 

        # allure.attach(data, 'Configuration', allure.attachment_type.CSV)            