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

import pytest

class PropertyBasedLanguageGenerator():

    def __init__(self):
        """
        self.scenario_composite_tests[0]: Contains standard information
        self.scenario_composite_tests[1]: Contains standard sections information
        self.scenario_composite_tests[2][0]: Contains section scenario information
        self.scenario_composite_tests[2][1]: Contains section composite properties information 
        """

        self.config = Configuration()               
        
        try:
            self.pblg = metamodel_from_file(self.config.grammar_dir)
        except:
            print('\n Test grammar rules directory not detected at {}'.format(self.config.grammar_dir))

        try:
            self.test_model = self.pblg.model_from_file(self.config.test_def_dir)
        except:
            print('\n Test definitions directory not detected at {}'.format(self.config.test_def_dir))

        try:
            self.standard = self.test_model.test_type[0].standard
            self.standard_sections = []

            for i in range(len(self.test_model.test_type[0].section_number)):
                self.standard_sections.append(self.test_model.test_type[0].section_number[i].section)
        except:
            print('No standard detected')

        # Standard Test Generator
        self.scenario_composite_tests = []
        
        for x in range(len(self.test_model.test_type[0].section_number)):

            scenario_compo = self.scenario_composite_test_extractor(
                                                self.test_model.test_type[0].section_number[x].scenario_configuration,
                                                self.test_model.test_type[0].section_number[x].custom_scenario.property_check)
            test_details = [self.standard, self.standard_sections[x], scenario_compo]

            self.scenario_composite_tests.append(test_details)

    def scenario_composite_test_extractor(self, scenario, composite_tests):

        comp = []
        scen = scenario

        for i in range(len(composite_tests)):
            
            if composite_tests[i].must_not_collide != None:
                comp.append(('must_not_collide', composite_tests[i].must_not_collide))

            if composite_tests[i].must_collide != None:
                comp.append(('must_collide', composite_tests[i].must_collide))

            if composite_tests[i].must_be_at != None:
                comp.append(('must_be_at', composite_tests[i].must_be_at))

            if composite_tests[i].must_not_be_at != None:
                comp.append(('must_not_be_at', composite_tests[i].must_not_be_at))

            if composite_tests[i].must_have_collision_force_less_than != None:
                comp.append(('must_have_collision_force_less_than', composite_tests[i].must_have_collision_force_less_than))

            if composite_tests[i].must_have_orientation != None:
                comp.append(('must_have_orientation', composite_tests[i].must_have_orientation))

            if composite_tests[i].must_be_near_to != None:
                comp.append(('must_be_near_to', composite_tests[i].must_be_near_to))

            if composite_tests[i].must_not_be_near_to != None:
                comp.append(('must_not_be_near_to', composite_tests[i].must_not_be_near_to))

        return (scen, comp)
             
# p = PropertyBasedLanguageGenerator()