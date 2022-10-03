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

from property_based_tester.configuration.config import Configuration
from property_based_tester.property_based_language_generation.textx_test_specification import PropertyBasedLanguageGenerator
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

pblg = PropertyBasedLanguageGenerator()
pblg_config = pblg.scenario_composite_tests 

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
        
        world = Model(pblg_config[2][0].world_type,0,0,0)
        world.insert_model()

        # Correcting robot spawn location
        try:
            robo = RobotModel(self.config.robot_urdf,
                            x= pblg_config[2][0].scenario_modifier[0].sm_robot_position[0].x_pos,
                            y= pblg_config[2][0].scenario_modifier[0].sm_robot_position[0].y_pos,
                            z= pblg_config[2][0].scenario_modifier[0].sm_robot_position[0].z_pos,
                            R= pblg_config[2][0].scenario_modifier[0].sm_robot_position[0].r_ori,
                            P= pblg_config[2][0].scenario_modifier[0].sm_robot_position[0].p_ori,
                            Y= pblg_config[2][0].scenario_modifier[0].sm_robot_position[0].y_ori)
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
            payload = Model(pblg_config[2][0].scenario_modifier[0].sm_payload[0].payload,
                            x=pblg_config[2][0].scenario_modifier[0].sm_payload[0].x_pos,
                            y=pblg_config[2][0].scenario_modifier[0].sm_payload[0].y_pos,
                            z=pblg_config[2][0].scenario_modifier[0].sm_payload[0].z_pos,
                            R= pblg_config[2][0].scenario_modifier[0].sm_payload[0].r_ori,
                            P= pblg_config[2][0].scenario_modifier[0].sm_payload[0].p_ori,
                            Y= pblg_config[2][0].scenario_modifier[0].sm_payload[0].y_ori)
            payload.insert_model()
        except:
            pass

        # Correcting safety_obstacle spawn
        try:
            for i in range(len(pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle)):
                safety_obstacle = Model(pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].safety_obstacle,
                                        pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].x_pos,
                                        pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].y_pos,
                                        pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].z_pos,
                                        pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].r_ori,
                                        pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].p_ori,
                                        pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].y_ori)
                safety_obstacle.insert_model()
        except:
            pass
            
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

    def test_scenario_execution(self, randomizer, pblg_config): 
        """Defines a scenario for the rest of the tests to run in using coodrinates.
        """    
        multi_goal = []
        result = None
        
        try:
            # Test definition goal if exists
            coord_x = pblg_config[2][0].scenario_modifier[0].sm_robot_goal[0].x_pos
            coord_y = pblg_config[2][0].scenario_modifier[0].sm_robot_goal[0].y_pos
            direction = pblg_config[2][0].scenario_modifier[0].sm_robot_goal[0].y_ori
        except:
            # Randomize given coordinates
            coord_x, coord_y, direction = randomizer(-2,2),randomizer(-2,2),randomizer(0,360)
         
        # Excute navigation and temporal logger
        temporal_logger = subprocess.Popen(['rosrun', self.config.rospkg_name, 'temporal_log.py'], preexec_fn=os.setsid)

        # Apply random forces
        try:
            apply_force(x=randomizer(-pblg_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces,
                            pblg_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces),
                        y=randomizer(-pblg_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces,
                            pblg_config[2][0].scenario_modifier[0].sm_imparted_forces[0].imparted_forces),
                        z=0,
                        link= pblg_config[2][0].scenario_modifier[0].sm_imparted_forces[0].target_entity,
                        timeout=5)
        except:
            pass

        try:
            # Direct control test
            if pblg_config[2][0].scenario_modifier[0].sm_robot_velocity[0].manual_speed == True:
                result = move(pblg_config[2][0].scenario_modifier[0].sm_robot_velocity[0].robot_speed,
                              self.config.robot_cmd_vel, 
                              pblg_config[2][0].scenario_modifier[0].sm_robot_velocity[0].speed_duration)
        except:
            for goals in pblg_config[2][0].scenario_modifier[0].sm_robot_goal:
                multi_goal.append([goals.x_pos, goals.y_pos, goals.y_ori])
            # Multi goal movebase control test
            for goal in multi_goal:
                # Create visual marker for final navigation point with no params
                finish_point = Model('flag', x= goal[0], y= goal[1], z= 0.0, R= 0, P= 0, Y= 0)
                finish_point.insert_model(placement_frame=self.config.robot_urdf)    
                result = single_goal_movebase(goal[0], goal[1], goal[2], timeout=10)
                # Remove finish marker
                delete_model('flag') 

        if result == None:
            # Create visual marker for final navigation point with no params
            finish_point = Model('flag', x= coord_x, y= coord_y, z= 0.0, R= 0, P= 0, Y= 0)
            finish_point.insert_model(placement_frame=self.config.robot_urdf)    
            result = single_goal_movebase(coord_x, coord_y, direction, timeout=10)   
            # Remove finish marker
            delete_model('flag') 
                    
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
        for configuration in pblg_config[2][1]:
            if configuration[0] == 'must_have_collision_force_less_than':
                check = True
                assert self.composite_properties.must_have_collision_force_less_than(pytest.collision_force, configuration[1].force_threshhold) == True
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_collide(self, pblg_config):
        """ Checking if the robot collided with a specific obstacle during navigation.
        """    
        check = False
        for configuration in pblg_config[2][1]:
            if configuration[0] == 'must_collide':
                check = True
                assert pytest.collision == True
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_not_collide(self, pblg_config):
        """ Checking if the robot has not collided with an obstacle during navigation.
        """ 
        check = False
        for configuration in pblg_config[2][1]:
            if configuration[0] == 'must_not_collide':
                check = True
                assert pytest.collision == False
        if check == False:
            pytest.skip("Test Un-marked")

    def test_must_be_at(self, pblg_config):
        """ Checking if the robot is within a given area.
        """    
        check = False
        for configuration in pblg_config[2][1]:
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
        for configuration in pblg_config[2][1]:
            if configuration[0] == 'must_not_be_at':
                check = True

                assert self.composite_properties.must_be_at(target_area_min=[configuration[1].x1,configuration[1].y1,configuration[1].z1], 
                                                            target_area_max=[configuration[1].x2,configuration[1].y2,configuration[1].z2]) == True
                
        if check == False:
            pytest.skip("Test Un-marked")
    
    def test_must_have_orientation(self, pblg_config):
        """ Checking if the robot is has a given area.
        """    
        check = False
        for configuration in pblg_config[2][1]:
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
        delete_model(pblg_config[2][0].world_type) 
        
        try:
            for i in range(len(pblg_config[2][0].scenario_modifier[0].sm_payload)):
                delete_model(pblg_config[2][0].scenario_modifier[0].sm_payload[i].payload) 
        except:
            pass

        try:
            for i in range(len(pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle)):
                delete_model(pblg_config[2][0].scenario_modifier[0].sm_safety_obstacle[i].safety_obstacle) 
        except:
            pass

        pytest.robot_controller.terminate()
        # allure.attach(data, 'Configuration', allure.attachment_type.CSV)            