#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Property-Based Language Generator based on textX.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 16, 2022
----------------------------------------------------
"""

from textx import metamodel_from_file
from property_based_tester.configuration.config import Configuration

class PropertyBasedLanguageGenerator():

    def __init__(self):

        self.config = Configuration()
        
        
        self.pblg = metamodel_from_file(self.config.grammar_dir)
        self.test_model = self.pblg.model_from_file(self.config.test_def_dir)

        self.robot_name = self.test_model.robot.robot_name
        self.robot_type = self.test_model.robot.robot_type
        self.standards = self.test_model.test_type[0].standard

    def translate_standards(self, model):
        print(model.robot.robot_name)
        print(model.robot.robot_type)

        print()

p = PropertyBasedLanguageGenerator()