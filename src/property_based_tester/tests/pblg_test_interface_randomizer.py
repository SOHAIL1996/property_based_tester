#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Property-Based Test

Tests robots in diverse scenarios.
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
import time

from property_based_tester.configuration.config import Configuration
from property_based_tester.property_based_language_generation.textx_test_specification import PropertyBasedLanguageGeneratorRandomizer
from property_based_tester.robot_controllers.navigation_client import single_goal_movebase
from property_based_tester.robot_controllers.speed_client import move
from property_based_tester.scen_gen.obstacle_gen import Model
from property_based_tester.scen_gen.force_generation import apply_force
from property_based_tester.scen_gen.model_placement import delete_model
from property_based_tester.scen_gen.robot_placement import RobotModel
from property_based_tester.properties.composite_properties import CompositeProperties

from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

global spawned_items
spawned_items = []

pblg = PropertyBasedLanguageGeneratorRandomizer()
pblg_config = pblg.rando_scenario  

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
        self.composite_properties = CompositeProperties()

@pytest.mark.usefixtures('set_up')    
@pytest.mark.parametrize('pblg_config', pblg_config, scope="class")
class TestScenario(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters

    def pytest_configure():
        pytest.collision = False
        pytest.collider_1 = None
        pytest.collider_2 = None
        pytest.collision_force = None
        pytest.robot_controller = None

    def test_set_up(self, pblg_config):
        """Initializing property-based generator language scenario.
        """  
        rospy.init_node('pblg_test')

        # Restarting robot control at start of each test
        pytest.robot_controller = subprocess.Popen(['roslaunch', self.config.rospkg_name, self.config.launch_controller_file])

    def test_scenario_generation(self, randomizer, pblg_config):
        
        print(pblg_config[0])
        world = Model(pblg_config[0],0,0,0)
        world.insert_model()

        robo = RobotModel(self.config.robot_urdf,
                        x= 0,
                        y= 0,
                        z= 0.12,
                        R= 0,
                        P= 0,
                        Y= 0)
        robo.robot_pose()

        # Correcting safety_obstacle spawn
        store = [[0,0],[0,1],[1,0],[0,-1],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1]]
        is_spawned = False
        for o in range(0,5):
            x,y = randomizer(-5,5), randomizer(-5,5)
            for i in store:
                if [x,y] == i:
                    is_spawned = True
                    break
            if is_spawned != True:
                store.append([x,y]) 
                safety_obstacle = Model(random.choice(pblg_config[1]),x, y, z=0.05, Y=randomizer(-360,360))
                safety_obstacle.insert_model()
            
        assert True

    def test_scenario_execution(self, randomizer, pblg_config): 
        """Defines a scenario for the rest of the tests to run in using coodrinates.
        """           
        coord_x, coord_y, direction = randomizer(-3,3),randomizer(-3,3),randomizer(0,360)
         
        # Excute navigation and temporal logger
        temporal_logger = subprocess.Popen(['rosrun', self.config.rospkg_name, 'temporal_log.py'], preexec_fn=os.setsid)
        result = single_goal_movebase(coord_x, coord_y, direction, timeout=10)   
                    
        os.killpg(os.getpgid(temporal_logger.pid), signal.SIGTERM) 
               
        # Logging test collision details for other tests
        pytest.collision = self.composite_properties.in_collision
        pytest.collider_1 = self.composite_properties.collider_1
        pytest.collider_2 = self.composite_properties.collider_2
        pytest.collision_force = self.composite_properties.collision_force
    
        assert result == True    

    def test_must_have_collision_force_less_than(self, pblg_config):
        """ Checking the max force exerted by the robot on an obstacle.
        """ 
        check = False
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_have_collision_force_less_than':
                check = True
                assert self.composite_properties.must_have_collision_force_less_than(pytest.collision_force, configuration[1].force_threshhold) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_collide(self, pblg_config):
        """ Checking if the robot collided with a specific obstacle during navigation.
        """    
        check = False
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_collide':
                check = True
                assert pytest.collision == True
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_not_collide(self, pblg_config):
        """ Checking if the robot has not collided with an obstacle during navigation.
        """ 
        check = False
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_not_collide':
                check = True
                assert pytest.collision == False
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_be_at(self, pblg_config):
        """ Checking if the robot is within a given area.
        """    
        check = False
        for configuration in pblg_config[2]:
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
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_not_be_at':
                check = True

                assert self.composite_properties.must_be_at(target_area_min=[configuration[1].x1,configuration[1].y1,configuration[1].z1], 
                                                            target_area_max=[configuration[1].x2,configuration[1].y2,configuration[1].z2]) == True
                
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_be_near_to(self, pblg_config):
        """ Checking if the robot is within a given euclidean distance.
        """    
        check = False
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_be_near_to':
                check = True
                assert self.composite_properties.must_be_near_to(object=self.config.robot_urdf, 
                                                                target_object=configuration[1].entity, 
                                                                req_dis=configuration[1].euclidean_distance, 
                                                                tolerance=configuration[1].tolerance) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_not_be_near_to(self, pblg_config):
        """ Checking if the robot is within a given euclidean distance.
        """    
        check = False
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_not_be_near_to':
                check = True
                assert self.composite_properties.must_be_near_to(object=self.config.robot_urdf, 
                                                                target_object=configuration[1].entity, 
                                                                req_dis=configuration[1].euclidean_distance, 
                                                                tolerance=configuration[1].tolerance) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_have_orientation(self, pblg_config):
        """ Checking if the robot is has a given area.
        """    
        check = False
        for configuration in pblg_config[2]:
            if configuration[0] == 'must_have_orientation':
                check = True
                assert self.composite_properties.must_have_orientation(object=configuration[1].entity, 
                                                                       orientation=[configuration[1].roll,
                                                                                    configuration[1].pitch,
                                                                                    configuration[1].yaw], 
                                                                       time=0, 
                                                                       tolerance=configuration[1].tolerance) == True
        if check == False:
            pytest.skip("Test Un-marked")

    def test_static_obstacle_generation_tear_down(self, pblg_config):
        """Tearing down the setup for navigation.
        """  
        delete_model(pblg_config[0]) 

        try:
            for i in pblg_config[1]:
                delete_model(i) 
        except:
            pass

        pytest.robot_controller.terminate()

        # Attaching log file to the test results
        test_parameters = self.config.workspace + '/src/property_based_tester/property_based_language_generation/test_definitions.pblg'
        simulation_data = self.config.workspace + '/src/property_based_tester/temporal_cache/logs/test_logs.csv'
        controller_parameters = self.config.workspace + '/src/property_based_tester/configuration/property_based_tester_params.yaml'

        with open(test_parameters) as f:
            test_data = f.read()
        
        with open(simulation_data) as f:
            sim_data = f.read()

        with open(controller_parameters) as f:
            controller_data = f.read()

        allure.attach(test_data, 'Test Parameters')
        allure.attach(sim_data, 'Simulation Data')
        allure.attach(controller_data, 'Controller Parameters')            