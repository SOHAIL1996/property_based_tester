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
import subprocess
import pytest
import allure
import time

from property_based_tester.configuration.config import Configuration
from property_based_tester.tests.action_client.navigation_client import pose_action_client
from property_based_tester.tests.obstacle_generator.obstacle_gen import Model
from property_based_tester.scen_gen.model_placement import model_placement
from property_based_tester.scen_gen.robot_placement import RobotModel

from property_based_tester.properties.composite_properties import CompositeProperties
from property_based_language_generation.textx_test_specification import PropertyBasedLanguageGenerator

from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

global spawned_items
spawned_items = []

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
        self.textx = PropertyBasedLanguageGenerator()
        self.composite_properties = CompositeProperties()
        
@pytest.mark.usefixtures('set_up')         
class TestNavigation(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters

    @pytest.fixture()
    def world_spawn(self):        
        model_placement()

    @pytest.fixture()
    def robo_spawn(self):
        robo = RobotModel(self.config.robot_urdf,x=0,y=0,z=0.1,R=0,P=0,Y=0)
        robo.robot_pose()

    def pytest_configure():
        pytest.robot_node = 0
        pytest.collision = False

    def get_selected_standards():
        return [['ISO 23482-1','7.2'],['ISO 3691','4.8']]

    def get_user_tests():
        return [['ISO 23482-1','7.2'],['ISO 3691','4.8']]

    def test_set_up(self, robo_spawn):
        """Initializing navigation scenario.
        """  
        rospy.init_node('nav_test')

    def test_static_obstacle_generation(self, randomizer):
        store = [[0,0]]
        is_spawned = False
        for i in range(int(self.config.model_obst_num)):
            x,y = randomizer(-3,3), randomizer(-3,3)
            for i in store:
                if [x,y] == i:
                    is_spawned = True
                    break
            if is_spawned != True:
                store.append([x,y])    
                obstacle_name = random.choice(self.config.obstacles)
                obstacles = Model(obstacle_name, x=x, y=y, z=0.02)
                obstacles.model_number = str(randomizer(1,1000))
                spawned_items.append(obstacle_name + obstacles.model_number)
                obstacles.insert_model()
                is_spawned = False

    @pytest.mark.parametrize("standard, section", get_selected_standards())
    def test_standard(self, standard, section):
        pytest.skip("unsupported configuration")
    
    # @settings(max_examples=1)
    # @given(st.sampled_from(['table','shelf','cabinet','sofa']))
    # def test_scenario_generation_map(destination): 
    #     """Defines a scenario for the rest of the tests to run in using navigation map.
    #     """    
    #     data_logger('logger/logs/nav_start')
    #     result = navi_action_client(destination)
    #     data_logger('logger/logs/nav_end')
    #     assert result == True

    # def test_verification_of_navigation(self, randomizer): 
    #     """Defines a scenario for the rest of the tests to run in using coodrinates.
    #     """    
    #     coord_x, coord_y, direction = randomizer(-2,2),randomizer(-2,2),randomizer(0,360)
    #     temporal_logger = subprocess.Popen(['rosrun', self.config.rospkg_name, 'temporal_nav_log.py'])
    #     result = pose_action_client(coord_x, coord_y, direction)
    #     temporal_logger.terminate() 
    #     pytest.collision = self.composite_properties.in_collision
    #     assert result == True    

    def test_collision_detection(self):
        """ Checking if the position of objects changed furing navigation i.e. Lucy collided with an obstacle.
        """    
        assert pytest.collision == False

    # def test_location_verification(self):
    #     """Checking if the projected position of the robot matches 
    #     the position in the simulator.
    #     """    
    #     hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2]
    #     omni = OmniListener()
    #     omni.omnibase_listener()
    #     x,y,z = omni.x, omni.y, omni.z
    #     assert hx-0.45 <= x <= hx+0.45
    #     assert hy-0.45 <= y <= hy+0.45
        
        
    # def test_destination_verification(self):
    #     """Checking if the projected position of the robot matches 
    #     the position in the simulator.
    #     """    
    #     hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2] 
    #     assert hx-0.45 <= destination_coord[0] <= hx+0.45
    #     assert hy-0.45 <= destination_coord[1] <= hy+0.45

    # def test_operation_zone_verification(self):
    #     """Checking if the projected position of the robot has not gone out of the boundary. 
    #     The boundary is the test lab navigation map and its dimensions are 10x10 m^2.
    #     """    
    #     hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2]
    #     omni = OmniListener()
    #     omni.omnibase_listener()
    #     x,y,z = omni.x, omni.y, omni.z
    #     assert -5 <= hx <= 5
    #     assert -5 <= hy <= 5
    #     assert -5 <= x <= 5
    #     assert -5 <= y <= 5   
    
    def test_static_obstacle_generation_tear_down(self):
        """Tearing down the setup for navigation.
        """  
        test = Model('glass') 
        for i in spawned_items:  
            test.delete_model(i)  

        # allure.attach(data, 'Configuration', allure.attachment_type.CSV)            