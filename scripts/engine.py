#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Engine

Starter module for the test suite.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""
import time
import subprocess

from property_based_tester.configuration.config import Configuration
from property_based_tester.scen_gen.model_placement import model_placement

from termcolor import colored

print(colored('Starting Automated Testing Gazebo Simulation', 'green'))

try:
    conf = Configuration()
    # robot_node = subprocess.Popen(['roslaunch', conf.rospkg_name, conf.launch_file])
    model_placement()
finally:
    time.sleep(1000)
    # robot_node.terminate() 
    print(colored('Terminating ros!','red'))    
