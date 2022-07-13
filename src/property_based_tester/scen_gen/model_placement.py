#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Model Placement

A launching file for static and dynamic model placement.
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
import numpy as np

import rospy
from property_based_tester.scen_gen.random_scenario_generator import World
from property_based_tester.scen_gen.random_scenario_generator import Model
from property_based_tester.logger.data_logger import data_logger
from property_based_tester.configuration.config import Configuration
from property_based_tester.configuration.config import choices
from property_based_tester.configuration.config import collision_checker

from termcolor import colored

def model_placement(param=None):
    """This just places some static objects when the world starts.
    It also logs the placement of the objects as startup_logs in the
    logger file.

    Args:
        param ([type], optional): [description]. Defaults to None.
    """    

    rospy.init_node('model_manipulator')

    conf = Configuration()
    dynamic_model_tracer = []
    
    # Static model placement
    static_model = Model(conf.world,0,0,0)
    static_model.insert_model()
      
    static_model_platform = Model('table',0,4,0.0254,0,0,0)
    static_model_platform.insert_model()
    
    static_model_shelf = Model('shelf',4.8, 3, 0.0254, 0, 0, 1.5708)
    static_model_shelf.insert_model()
    
    static_model_sofa = Model('sofa',-3.7, -4.8, 0.0254, 0, 0, 1.5708)
    static_model_sofa.insert_model()
    
    static_model_cabinet = Model('cabinet',4.5, -4.8, 0.0254, 0, 0, 1.5708)
    static_model_cabinet.insert_model()
    
    # Temporary fixed spawning until MAS develops perceive pick library for Lucy
    # complex_scenario_bottle = Model('glass',0.000, 0.900, 0.500, 0, 0, 0)
    # complex_scenario_bottle.insert_model()
    # dynamic_model_tracer.append(complex_scenario_bottle)

    # Dynamic obstacles placement
    model_choices = choices(conf.models_manip, k=int(conf.model_manip_num))
    # Dynamic model placement through testing
    # model_choices = choices(conf.model_list(), k=int(param))
    
    # Ensuring the models spawns in the vicinity of the static model platform in this case the table
    minx = np.around(static_model_platform.bounding_box[0]*0.8+float(static_model_platform.x_coord),2)
    maxx = np.around(static_model_platform.bounding_box[1]*0.8+float(static_model_platform.x_coord),2)
    miny = np.around(static_model_platform.bounding_box[2]*0.8+float(static_model_platform.y_coord),2)
    maxy = np.around(static_model_platform.bounding_box[3]*0.8+float(static_model_platform.y_coord),2)
    minz = np.around(static_model_platform.bounding_box[4]+float(static_model_platform.z_coord),2)
    maxz = np.around(static_model_platform.bounding_box[5]+float(static_model_platform.z_coord),2)

    for model in model_choices:
        dynamic_model = Model(model)
                                
        # Ensuring models don't spawn on top of existing models
        for iteration in range(0,1000):
            
            x = random.uniform(minx, maxx)
            y = random.uniform(miny, maxy)
            z = maxz + 0.015

            check = collision_checker(dynamic_model,x,y,z,dynamic_model_tracer)
            if check == False:
                dynamic_model.x_coord = x
                dynamic_model.y_coord = y
                dynamic_model.z_coord = z
                
                dynamic_model.insert_model()
                dynamic_model_tracer.append(dynamic_model)
                break
            
            if iteration == 999:
                print(colored('No space for model insertion ','red'),dynamic_model.model_real_name)
                continue
    print(colored('Completed spawning dynamic models','cyan'))
    data_logger('src/logger/logs/start_up')