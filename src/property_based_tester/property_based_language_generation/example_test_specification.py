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

class PropertyBasedLanguageGenerator():

    def __init__(self):
        self.x = 0
        self.y = 0

    def interpret(self, model):
        print(model.robot.robot_name)
        print(model.robot.robot_type)

        print(model.test_type[0].standard)

if __name__ == '__main__':

    GRAMMAR_DIR = '/home/sorox23/robotic_ws/master_thesis_ws/src/property_based_tester/src/property_based_tester/property_based_language_generation/rules.tx'
    TEST_DEF_DIR = '/home/sorox23/robotic_ws/master_thesis_ws/src/property_based_tester/src/property_based_tester/property_based_language_generation/standard_test_definitions.pblg'

    # Importing rules and test definitons
    pblg = metamodel_from_file(GRAMMAR_DIR)
    test_model = pblg.model_from_file(TEST_DEF_DIR)

    # Instantiating for use
    pblg = PropertyBasedLanguageGenerator()
    pblg.interpret(test_model)