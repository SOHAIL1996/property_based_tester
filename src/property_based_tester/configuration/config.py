#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Utilities

Provides miscellaneous utility functions.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""

import yaml
import random
import os
import numpy as np
from stl import mesh
from itertools import accumulate as _accumulate, repeat as _repeat
from bisect import bisect as _bisect
import rospkg


class Configuration():
    """It extracts information from the configuration file.
    """    
    def __init__(self):
        
        self.rospkg_name = 'property_based_tester'

        self.root_dir = os.path.dirname(os.path.abspath(__file__))
        self.workspace = rospkg.RosPack().get_path(self.rospkg_name)
        self.model_dir = self.workspace +'/environment/models'
        self.robots_dir = self.workspace +'/environment/robots'

        self.grammar_dir = self.workspace +'/src/property_based_tester/property_based_language_generation/rules.tx'
        self.test_def_dir = self.workspace +'/src/property_based_tester/property_based_language_generation/standard_test_definitions.pblg'

        self.yaml_parser()

    def yaml_parser(self):
        
        yaml_path = os.path.join(self.root_dir, 'property_based_tester_params.yaml')

        stream = open(yaml_path, 'r')
        dictionary = yaml.full_load(stream)

        for key, value in dictionary.items():

            if key == "World":
                self.world = value

            if key == "Robot":
                for key_ro, value_ro in value.items():
                    if key_ro == "robot_urdf_name":
                        self.robot_urdf = value_ro
                    if key_ro == "robot_spawner_name":
                        self.launch_robot_file = value_ro
                    if key_ro == "robot_controller":
                        self.launch_controller_file = value_ro

            if key == "Models_manipulation":
                for key_manip, value_manip in value.items():
                    if key_manip == "Spawnable":
                        self.model_manip_num = value_manip
                    if key_manip == "Objects":
                        self.models_manip = value_manip
            
            if key == "Models_obstacles":
                for key_obst, value_obst in value.items():
                    if key_obst == "Spawnable":
                        self.model_obst_num = value_obst
                    if key_obst == "Objects":
                        self.obstacles = value_obst

    def model_uri_correcter(self, model_name):
        """
        This function opens the sdf model files and corrects the uri tags. This is useful when changing the
        paths of the model folder. 

        Args:
            model_name (str): The model name for the uri tag correction.
        """        
        with open(self.model_dir + model_name+'/'+model_name+'.sdf',mode='r') as file:
                counter    = 0
                data       = file.readlines()
                save_index = []

                for word in data:
                    if (word.find('uri') != -1): 
                        save_index.append(counter) 
                    counter +=1
                
                data[save_index[0]] = '             <uri>'+self.model_dir+'/'+model_name+'/meshes/'+model_name+'.stl</uri>\n'
                data[save_index[1]] = '             <uri>'+self.model_dir+'/'+model_name+'/meshes/'+model_name+'.stl</uri>\n'
                data[save_index[2]] = '             <uri>'+self.model_dir+'/'+model_name+'/materials/scripts</uri>\n'
                data[save_index[3]] = '             <uri>'+self.model_dir+'/'+model_name+'/materials/textures</uri>\n'
        
        with open(self.model_dir + model_name+'/'+model_name+'.sdf',mode='w') as file:
            file.writelines(data)
            
    
def mesh_extractor(model):
    """Extracts mesh from the model folder.
    """    
    conf = Configuration()
    model_directory = conf.workspace +'/environment/models/'+ model +'/meshes/'+ model +'.stl'
    model_mesh = mesh.Mesh.from_file(model_directory)
    return model_mesh
    
def find_mins_maxs(obj):
    """This extracts the minimum and maximum value of the meshes boundaries.

    Args:
        obj 
        class.obj: A class object obtained from the numpy-stl library. The 
        functionality is directly tied to the mesh_extracto function.
        

    Returns:
        minx, maxx, miny, maxy, minz, maxz 
        float: The values of the cartesian coordinate.
    """   
    minx = obj.x.min()
    maxx = obj.x.max()
    miny = obj.y.min()
    maxy = obj.y.max()
    minz = obj.z.min()
    maxz = obj.z.max()
    return minx, maxx, miny, maxy, minz, maxz

def bound_box(model_name):
    """Generates bounding box for a given model.
    """    
    mesh_obj = mesh_extractor(model_name)
    cartesian_min_max_vals = find_mins_maxs(mesh_obj)
    adjusting_for_scaling = np.asarray(cartesian_min_max_vals)*0.001
    return adjusting_for_scaling   

def stl_property_extractor(model_name):
    """Views the property of a specific mesh, doesnt work well.
    """    
    meshy = mesh_extractor(model_name)
    volume, cog, inertia = meshy.get_mass_properties()
    print("Volume                                  = {0}".format(volume))
    print("Position of the center of gravity (COG) = {0}".format(cog))
    print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
    print("                                          {0}".format(inertia[1,:]))
    print("                                          {0}".format(inertia[2,:]))
    print(inertia/volume)
    

def choices(population, weights=None, *, cum_weights=None, k=1):
    """Return a k sized list of population elements chosen with replacement.
    If the relative weights or cumulative weights are not specified,
    the selections are made with equal probability.
    """
    n = len(population)
    if cum_weights is None:
        if weights is None:
            _int = int
            n += 0.0    # convert to float for a small speed improvement
            return [population[_int(random.random() * n)] for i in _repeat(None, k)]
        cum_weights = list(_accumulate(weights))
    elif weights is not None:
        raise TypeError('Cannot specify both weights and cumulative weights')
    if len(cum_weights) != n:
        raise ValueError('The number of weights does not match the population')
    bisect = _bisect
    total = cum_weights[-1] + 0.0   # convert to float
    hi = n - 1
    return [population[bisect(cum_weights, random.random() * total, 0, hi)]
            for i in _repeat(None, k)]
    
def collision_checker(model,x,y,z,model_tracer):
    """Checks the collision with all other dynamic models. Uses 
    Axis-Aligned Bounding Box.

    Args:
        obj_small_x (float): Model's bounding box minimum x coordinate.
        obj_big_x (float): Model's bounding box maximum x coordinate.
        obj_small_y (float): Model's bounding box minimum y coordinate.
        obj_big_y (float): Model's bounding box maximum y coordinate.

    Returns:
        bool: Returns True if there is no valid position for given model.
              Returns False if there is a valid position for given model.
    """    
    # Model to be placed bounding box
    org_sx = np.around(model.bounding_box[0]+x,2)+0.6
    org_bx = np.around(model.bounding_box[1]+x,2)+0.6
    org_sy = np.around(model.bounding_box[2]+y,2)+0.6
    org_by = np.around(model.bounding_box[3]+y,2)+0.6
    org_sz = np.around(model.bounding_box[4]+z,2)+0.6
    org_bz = np.around(model.bounding_box[5]+z,2)+0.6
        
    if model_tracer != []: 
             
        for models in np.arange(len(model_tracer)): 
            # Previous models bounding box
            com_sx = np.around(model_tracer[models].bounding_box[0]+float(model_tracer[models].x_coord),2)+0.6
            com_bx = np.around(model_tracer[models].bounding_box[1]+float(model_tracer[models].x_coord),2)+0.6
            com_sy = np.around(model_tracer[models].bounding_box[2]+float(model_tracer[models].y_coord),2)+0.6
            com_by = np.around(model_tracer[models].bounding_box[3]+float(model_tracer[models].y_coord),2)+0.6
            com_sz = np.around(model_tracer[models].bounding_box[4]+float(model_tracer[models].z_coord),2)+0.6
            com_bz = np.around(model_tracer[models].bounding_box[5]+float(model_tracer[models].z_coord),2)+0.6
            
            # Collision X-Y-Z-element
            if (org_sx < com_bx and org_bx > com_sx and org_sy < com_by and org_by > com_sy and org_sz < com_bz and org_bz > com_sz):
                return True
            # Collision X-Y-element Axis Aligned Collision Box
            # if (org_sx < com_bx and org_bx > com_sx and org_sy < com_by and org_by > com_sy):
            #     return True
        return False
    else:
        return False  

if __name__ == '__main__':
    pass