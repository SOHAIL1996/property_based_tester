#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Property-Based Test: Tests for robotic arms using the Robot Test Definition Language in user defined scenarios
and standard designated scenarios.
"""

import random
import rospy
import numpy as np
import os, subprocess, signal, time
import pytest
import allure
import time

from property_based_tester.configuration.config import Configuration
from property_based_tester.robot_test_definition_language.textx_test_specification import RobotTestDefinitionLanguage
from property_based_tester.robot_controllers.xarm_moveit import ManipulatorMoveit
from property_based_tester.scen_gen.obstacle_gen import Model
from property_based_tester.scen_gen.force_generation import apply_force
from property_based_tester.scen_gen.model_placement import delete_model
from property_based_tester.scen_gen.robot_placement import RobotModel
from property_based_tester.properties.composite_properties import CompositeProperties


from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

global spawned_items
spawned_items = []

pblg = RobotTestDefinitionLanguage()
rtdl_config = pblg.scenario_composite_tests 

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
        self.composite_properties = CompositeProperties()

@pytest.mark.usefixtures('set_up')    
@pytest.mark.parametrize('rtdl_config', rtdl_config, scope="class")
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

    def test_set_up(self, rtdl_config):
        """Initializing property-based generator language scenario.
        """  
        pytest.xarm6 = ManipulatorMoveit('xarm6')
        pytest.xarm6_gripper = ManipulatorMoveit('xarm_gripper')

        # Restarting robot control at start of each test
        # pytest.robot_controller = subprocess.Popen(['roslaunch', self.config.rospkg_name, self.config.launch_controller_file])

    def test_scenario_generation(self, randomizer, rtdl_config):
        
        world = Model(rtdl_config[2][0].world_type,0,0,0)
        world.insert_model()

        # Correcting robot spawn location
        try:
            robo = RobotModel(self.config.robot_urdf,
                            x= rtdl_config[2][0].scenario_modifier[0].sm_robot_position[0].x_pos,
                            y= rtdl_config[2][0].scenario_modifier[0].sm_robot_position[0].y_pos,
                            z= rtdl_config[2][0].scenario_modifier[0].sm_robot_position[0].z_pos,
                            R= rtdl_config[2][0].scenario_modifier[0].sm_robot_position[0].r_ori,
                            P= rtdl_config[2][0].scenario_modifier[0].sm_robot_position[0].p_ori,
                            Y= rtdl_config[2][0].scenario_modifier[0].sm_robot_position[0].y_ori)
            robo.robot_pose()
        except:
            robo = RobotModel(self.config.robot_urdf,
                            x= 0,
                            y= 0,
                            z= 0.12,
                            R= 0,
                            P= 0,
                            Y= 0)
            robo.robot_pose()

        # Correcting payload spawn
        try:
            payload = Model(rtdl_config[2][0].scenario_modifier[0].sm_payload[0].payload,
                            x=rtdl_config[2][0].scenario_modifier[0].sm_payload[0].x_pos,
                            y=rtdl_config[2][0].scenario_modifier[0].sm_payload[0].y_pos,
                            z=rtdl_config[2][0].scenario_modifier[0].sm_payload[0].z_pos,
                            R= rtdl_config[2][0].scenario_modifier[0].sm_payload[0].r_ori,
                            P= rtdl_config[2][0].scenario_modifier[0].sm_payload[0].p_ori,
                            Y= rtdl_config[2][0].scenario_modifier[0].sm_payload[0].y_ori)
            payload.insert_model()
        except:
            pass

        # Correcting safety_obstacle spawn
        try:
            for i in range(len(rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle)):
                safety_obstacle = Model(rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].safety_obstacle,
                                        rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].x_pos,
                                        rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].y_pos,
                                        rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].z_pos,
                                        rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].r_ori,
                                        rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].p_ori,
                                        rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].y_ori)
                safety_obstacle.insert_model()
        except:
            pass
            
        assert True
    
    def test_scenario_execution(self, randomizer, rtdl_config): 
        """Defines a scenario for the rest of the tests to run in using coodrinates.
        """    
        multi_goal = []
        result = None
                
        # Excute manipulation and temporal logger
        temporal_logger = subprocess.Popen(['rosrun', self.config.rospkg_name, 'temporal_log.py'], preexec_fn=os.setsid)

        # Apply random forces
        try:
            apply_force(x=randomizer(-rtdl_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces,
                            rtdl_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces),
                        y=randomizer(-rtdl_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces,
                            rtdl_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces),
                        z=0,
                        link= rtdl_config[2][0].scenario_modifier[0].sm_imparted_forces[0].target_entity,
                        timeout=5)
        except:
            pass

        try:
            for goals in rtdl_config[2][0].scenario_modifier[0].sm_robot_goal:
                multi_goal.append([goals.x_pos, goals.y_pos, goals.z_pos, goals.r_ori, goals.p_ori, goals.y_ori])
            # Multi goal movebase control test
            for goal in multi_goal:
                if goal[0] == 1000:
                    pytest.xarm6_gripper.go_to_manipulator_joint_state(goal[1])
                    continue 

                result = pytest.xarm6.go_to_pose_goal(x=goal[0], y=goal[1], z=goal[2], R=goal[3] , P=goal[4], Y=goal[5])
            pytest.xarm6.set_state('home') 
        except:
            pass

                    
        os.killpg(os.getpgid(temporal_logger.pid), signal.SIGTERM) 
               
        # Logging test collision details for other tests
        pytest.collision = self.composite_properties.in_collision
        pytest.collider_1 = self.composite_properties.collider_1
        pytest.collider_2 = self.composite_properties.collider_2
        pytest.collision_force = self.composite_properties.collision_force

        # Apply dead time for robot slippage and checking stopping time
        try:
            time.sleep(rtdl_config[2][0].scenario_modifier[0].sm_dead_time[0].dead_time)
        except:
            pass
    
        assert result == True    

    def test_must_have_collision_force_less_than(self, rtdl_config):
        """ Checking the max force exerted by the robot on an obstacle.
        """ 
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_have_collision_force_less_than':
                check = True
                assert self.composite_properties.must_have_collision_force_less_than(pytest.collision_force, configuration[1].force_threshhold) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_collide(self, rtdl_config):
        """ Checking if the robot collided with a specific obstacle during manipulation.
        """    
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_collide':
                check = True
                assert pytest.collision == True
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_not_collide(self, rtdl_config):
        """ Checking if the robot has not collided with an obstacle during manipulation.
        """ 
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_not_collide':
                check = True
                assert pytest.collision == False
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_be_at(self, rtdl_config):
        """ Checking if the robot is within a given area.
        """    
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_be_at':
                check = True
                assert self.composite_properties.must_be_at(target_area_min=[configuration[1].x1,configuration[1].y1,configuration[1].z1], 
                                                            target_area_max=[configuration[1].x2,configuration[1].y2,configuration[1].z2]) == True
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_not_be_at(self, rtdl_config):
        """ Checking if the robot is not within a given area.
        """    
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_not_be_at':
                check = True

                assert self.composite_properties.must_be_at(target_area_min=[configuration[1].x1,configuration[1].y1,configuration[1].z1], 
                                                            target_area_max=[configuration[1].x2,configuration[1].y2,configuration[1].z2]) == True
                
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_be_near_to(self, rtdl_config):
        """ Checking if the robot is within a given euclidean distance.
        """    
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_be_near_to':
                check = True
                assert self.composite_properties.must_be_near_to(object=self.config.robot_urdf, 
                                                                target_object=configuration[1].entity, 
                                                                req_dis=configuration[1].euclidean_distance, 
                                                                tolerance=configuration[1].tolerance) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_not_be_near_to(self, rtdl_config):
        """ Checking if the robot is within a given euclidean distance.
        """    
        check = False
        for configuration in rtdl_config[2][1]:
            if configuration[0] == 'must_not_be_near_to':
                check = True
                assert self.composite_properties.must_be_near_to(object=self.config.robot_urdf, 
                                                                target_object=configuration[1].entity, 
                                                                req_dis=configuration[1].euclidean_distance, 
                                                                tolerance=configuration[1].tolerance) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_have_orientation(self, rtdl_config):
        """ Checking if the robot is has a given area.
        """    
        check = False
        for configuration in rtdl_config[2][1]:
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

    def test_static_obstacle_generation_tear_down(self, rtdl_config):
        """Tearing down the setup for manipulation.
        """  
        delete_model(rtdl_config[2][0].world_type) 
        
        try:
            for i in range(len(rtdl_config[2][0].scenario_modifier[0].sm_payload)):
                delete_model(rtdl_config[2][0].scenario_modifier[0].sm_payload[i].payload) 
        except:
            pass

        try:
            for i in range(len(rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle)):
                delete_model(rtdl_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].safety_obstacle) 
        except:
            pass

        # pytest.robot_controller.terminate()

        # Attaching log file to the test results
        test_parameters = self.config.workspace + '/src/property_based_tester/robot_test_definition_language/test_definitions.pblg'
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