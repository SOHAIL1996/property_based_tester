#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Tests compliant to  several standards as mentioned
in the README.md
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""


from property_based_language_generation.textx_test_specification import PropertyBasedLanguageGenerator

class UserTesting():

    def __init__(self) -> None:

        # self.textx = PropertyBasedLanguageGenerator()

        self.must_be_at = False
        self.must_not_be_at = False
        self.must_be_near_to = False
        self.must_be_near_to = False
        self.must_collide = True
        self.must_not_collide = True
