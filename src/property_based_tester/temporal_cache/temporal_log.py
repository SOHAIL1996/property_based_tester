#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Logs the temporal properties of the models.

Starter module for the source code.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""
import numpy as np
import pandas as pd
import time
from property_based_tester.temporal_cache.data_depot import data_logger, world_state_extractor
from property_based_tester.configuration.config import Configuration

if __name__ == '__main__':
    try:
        
        logs_objects = ['Models', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4','Time']
        logs_all = [['------','------','------','------','------','------','------','------ ','------']]
        config = Configuration()
        
        while True:
            logs = world_state_extractor()
            logs_all.extend(logs)

            time.sleep(2)

            df = pd.DataFrame(columns=logs_objects, data=logs_all[1:])
            data_logger(df, config.workspace+'/src/property_based_tester/temporal_cache/logs/test')
            logs_all.extend([['------','------','------','------','------','------','------','------ ','------']])
    finally:
        print('Error in file storage')       