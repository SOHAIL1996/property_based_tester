#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Property-Based Language Generator: A domain specific language based on textX for mapping
usecases into tests via natural language.
"""

from textx import metamodel_from_file
from property_based_tester.configuration.config import Configuration

import random

class PropertyBasedLanguageGenerator():
    """Pre-processing of the test definitions for user tests as well as standard tests.
    """

    def __init__(self):
        """
        self.scenario_composite_tests[0]:    Contains standard information
        self.scenario_composite_tests[1]:    Contains standard sections information
        self.scenario_composite_tests[2][0]: Contains section scenario information
        self.scenario_composite_tests[2][1]: Contains section composite properties information 
        """
        # Standard Test Generator
        self.scenario_composite_tests = []
        
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

            for x in range(len(self.test_model.test_type[0].section_number)):

                scenario_compo = self.scenario_composite_test_extractor(
                                                    self.test_model.test_type[0].section_number[x].scenario_configuration,
                                                    self.test_model.test_type[0].section_number[x].custom_scenario.property_check)
                test_details = [self.standard, self.standard_sections[x], scenario_compo]

                self.scenario_composite_tests.append(test_details)
        except:
            print('No standard detected using User Scenario')
            for x in range(len(self.test_model.test_type)):
                scenario_compo = self.scenario_composite_test_extractor(
                                                    self.test_model.test_type[x].scenario_configuration,
                                                    self.test_model.test_type[x].custom_scenario.property_check)
                test_details = ['User Scenario', ' ', scenario_compo]

                self.scenario_composite_tests.append(test_details)        

    def scenario_composite_test_extractor(self, scenario, composite_tests):
        """Pre-processing in which the composite properties are parsed and flagged with their 
        test types from the inputted test definitions. 

        Args:
            scenario (list): The scenario designated in the test definition file.
            composite_tests (list): The composite properties designate in the test definition file.

        Returns:
            tuple: Scenario Definition (list), Composite Properties (list)
        """

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

class PropertyBasedLanguageGeneratorRandomizer():
    """Pre-processing of the test definitions for randomized tests.
    """

    def __init__(self):

        self.scenario_composite_tests = []
        self.config = Configuration()               
        
        try:
            self.pblg = metamodel_from_file(self.config.grammar_dir)
            self.test_model = self.pblg.model_from_file(self.config.test_def_dir)
        except:
            print('\n Test definitions directory not detected at {}'.format(self.config.test_def_dir))

        
        scenario_compo = self.scenario_composite_test_extractor(
                                            self.test_model.test_type[0].scenario_configuration,
                                            self.test_model.test_type[0].custom_scenario.property_check)
        test_details = ['Random Scenario', ' ', scenario_compo]

        self.scenario_composite_tests.append(test_details)

        self.rando_scenario = self.random_scenario_generation()
        # print(self.rando_scenario[0][2])

    def scenario_composite_test_extractor(self, scenario, composite_tests):
        """Pre-processing in which the composite properties are parsed and flagged with their 
        test types from the inputted test definitions. 

        Args:
            scenario (list): The scenario designated in the test definition file.
            composite_tests (list): The composite properties designate in the test definition file.

        Returns:
            tuple: Scenario Definition (list), Composite Properties (list)
        """

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

    def random_scenario_generation(self):
        """Pre-processing in which the the randomized scenario modifiers from the test definitions 
        are extracted and returned. 

        Returns:
            list: Randomized scenario modifiers
        """

        random_scen = []
        comp_props = []
        worlds = []
        obst = []

        iter = self.scenario_composite_tests[0][2][0].iterations
        
        for i in range(len(self.scenario_composite_tests[0][2][1])):
            # print(self.scenario_composite_tests[0][2][1][i][0])
            comp_props.append(self.scenario_composite_tests[0][2][1][i])

        for i in range(len(self.scenario_composite_tests[0][2][0].worlds)):
            worlds.append(self.scenario_composite_tests[0][2][0].worlds[i].world_type)

        for i in range(len(self.scenario_composite_tests[0][2][0].obstacles)):
            obst.append(self.scenario_composite_tests[0][2][0].obstacles[i].obstacle_type)

        for i in range(iter):
            random_scen.append([random.choice(worlds), obst, comp_props])
        
        return random_scen

# p = PropertyBasedLanguageGeneratorRandomizer()