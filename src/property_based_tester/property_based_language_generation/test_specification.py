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


pblg = metamodel_from_file('rules.tx')
test_model = pblg.model_from_file('test_definitions.pblg')
