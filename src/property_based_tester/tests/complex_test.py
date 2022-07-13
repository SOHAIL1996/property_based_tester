#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Complex-tests module

Tests Lucy in a complex-scenarios.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""

import rospy
import numpy as np
import pytest
from termcolor import colored
import allure

from gazebo_msgs.srv import GetLightProperties

from property_based_tester.tests.action_client.nav_client import pose_action_client
from property_based_tester.tests.action_client.nav_client import toyota_action_client
from property_based_tester.utilities.Omni_base_locator.oml import OmniListener
from property_based_tester.tests.obstacle_generator.obstacle_gen import Model
from property_based_tester.tests.file_reader.file_reader import Configuration
from property_based_tester.logger.data_logger import log_reader_comparator
from property_based_tester.logger.data_logger import log_hsrb_reader

from hsrb_interface import Robot

from hypothesis import given, settings, Verbosity, example

from property_based_tester.tests.world_properties.world_prop import world_state
from property_based_tester.tests.action_client.perceive_client import perceive_client
from property_based_tester.tests.action_client.pick_place_client import MoveItPickAndPlace
from property_based_tester.tests.action_client.pick_client import picker_client
from property_based_tester.logger.data_logger import data_logger, lucy_gripper_information
from property_based_tester.logger.data_logger import data_reader, object_information

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

    def test_verification_of_navigation(self): 
        """Defines a scenario for the rest of the tests to run in using coodrinates.
        """    
        coord_x, coord_y, direction = 0.000, 2.855, 90
        # coord_x, coord_y, direction = 0.000, 0.000, 90
        destination_coord.append(coord_x)
        destination_coord.append(coord_y)
        data_logger('logger/logs/nav_start')
        result = pose_action_client(coord_x, coord_y, direction)
        data_logger('logger/logs/nav_end')
        assert result == True      

    def test_collision_detection(self):
        """ Checking if the position of objects changed furing navigation i.e. Lucy collided with an obstacle.
        """    
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('X-pos', 'nav_start', 'nav_end')
        assert lower_tolerance_difference == upper_tolerance_difference
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Y-pos', 'nav_start', 'nav_end')
        assert lower_tolerance_difference == upper_tolerance_difference
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Z-pos', 'nav_start', 'nav_end')
        assert lower_tolerance_difference == upper_tolerance_difference

    def test_location_verification(self):
        """Checking if the projected position of the robot matches 
        the position in the simulator.
        """    
        hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2]
        omni = OmniListener()
        omni.omnibase_listener()
        x,y,z = omni.x, omni.y, omni.z
        assert hx-0.25 <= x <= hx+0.25
        assert hy-0.25 <= y <= hy+0.25
        
        
    def test_destination_verification(self):
        """Checking if the projected position of the robot matches 
        the position in the simulator.
        """    
        hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2] 
        assert hx-0.25 <= destination_coord[0] <= hx+0.25
        assert hy-0.25 <= destination_coord[1] <= hy+0.25

    def test_operation_zone_verification(self):
        """Checking if the projected position of the robot has not gone out of the boundary. 
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
    
    def test_tear_down_nav(self):
        """Tearing down the setup for navigation.
        """  
        # Attaching log file to the test results
        logs = self.config.config_data_frame('nav_end')
        data = logs.to_csv(index=False)
        allure.attach(data, 'Configuration_nav', allure.attachment_type.CSV)    

    def test_verification_of_perception(self):
        """Activates perception action.
        """
        data_logger('logger/logs/perception')
    #     # MAS HSR perception library not configured, requires ROS to not shutdown conflicts
    #     result = perceive_client()    
    #     assert True == result
    
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
        """Verification if object is ahead.
        """
        rospy.wait_for_service('/gazebo/get_light_properties')
        try:
            light_prop = rospy.ServiceProxy('/gazebo/get_light_properties',GetLightProperties)
            light_props = light_prop('sun')
            color_spectrum = light_props.diffuse
        except rospy.ServiceException as e:
            print(colored('Cannot acquire ligh State.','red')) 
        assert (color_spectrum.r or color_spectrum.g or color_spectrum.b) > 0.5  
    
    def test_verification_of_pick_action(self):
        """Activating the pick action test and checking whether it was successful.
        """       
        data_logger('logger/logs/pick_action_start')
        attempt = 0
        while attempt<3:
            try:
                result = MoveItPickAndPlace(pick_x = -0.03, pick_y = 0.90 , pick_z = 0.56, 
                                            place_x = 0.5, place_y = 1.00 , place_z = 0.65)
                break
            except RuntimeError:   
                print('Runtime error retrying.')
                attempt = attempt + 1
        data_logger('logger/logs/pick_action_end')
       
    def test_collision_detection_pick(self):
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
        
    def test_tear_down_pick(self):
        """Tearing down the setup for the pick test.
        """  
        # Attaching log file to the test results
        logs = self.config.config_data_frame('pick_action_end')
        data = logs.to_csv(index=False)
        allure.attach(data, 'Configuration_pick', allure.attachment_type.CSV)