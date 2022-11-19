#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Composite Properties: Culmination of composite properties that can used.
"""

from cgitb import reset
from matplotlib import pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.patches import Rectangle

import numpy as np
import rospy
from stl import mesh
from scipy.spatial import distance

from property_based_tester.configuration.config import Configuration
from property_based_tester.properties.primitive_properties import PrimitiveProperties
from gazebo_msgs.msg import ContactsState

class CompositeProperties():

    def __init__(self) -> None:
        self.config = Configuration()
        self.primitive_properties = PrimitiveProperties()

        self.in_collision = False 
        self.collider_1 = False 
        self.collider_2 = False 
        self.collision_force = False 
        rospy.Subscriber("bumper_contact_state", ContactsState, self.robot_force_sensor_callback)

    def must_be_at(self, object='jackal_robot', target_area_min=[-2,-2,-2], target_area_max=[1, 2, 2], time=0, tolerance=0):
        """Composite property that ensures the robot is in its given operation zone througout the
            a given scenario.

        Args:
            target_area_min (list, optional): minimum coordinates of the bounding zone. Defaults to [-2,-2,-2].
            target_area_max (list, optional): maximum coordinates of the bounding zone. Defaults to [1, 2, 2].
            time (int, optional): time to check the entity (currently not implemented). Defaults to 0.
            tolerance (int, optional): Safety tolerance value that robot shouldn't exceed. Defaults to 0.

        Returns:
            Bool: Returns True if the robot stayed inside the operation zone.
        """
        all_robot_position = self.primitive_properties.robo_spatial_temporal_information(object)

        result = []

        for robot_position in all_robot_position:
            robot_size_min_x_coord = (-1*self.config.robot_size[0]/2) + tolerance + robot_position[0]
            robot_size_min_y_coord = (-1*self.config.robot_size[1]/2) + tolerance + robot_position[1]
            robot_size_min_z_coord = (-1*self.config.robot_size[2]/2) + tolerance + robot_position[2]

            robot_size_max_x_coord = (self.config.robot_size[0]/2) + tolerance + robot_position[0]
            robot_size_max_y_coord = (self.config.robot_size[1]/2) + tolerance + robot_position[1]        
            robot_size_max_z_coord = (self.config.robot_size[2]/2) + tolerance + robot_position[2]
                
            com_sx =  target_area_min[0]
            com_sy =  target_area_min[1]
            com_sz =  target_area_min[2]

            com_bx =  target_area_max[0]
            com_by =  target_area_max[1]
            com_bz =  target_area_max[2]
           
            # Operation zone
            if (robot_size_min_x_coord > com_sx and robot_size_max_x_coord < com_bx and robot_size_min_y_coord > com_sy and robot_size_max_y_coord < com_by):
                result.append(0)
            else:
                result.append(1)

        robot_at_designated_location = sum(result)
        
        if robot_at_designated_location > 0:
            return False
        else:
            return True
        
    def must_not_be_at(self, object='jackal_robot', target_area_min=[2, 2, -1], target_area_max=[6, 6, 2], time=0, tolerance=0):
        """Composite property that ensures the robot does not wander into a forbidden zone througout the
            a given scenario.
        Args:
            target_area_min (list, optional): minimum coordinates of the bounding zone. Defaults to [2, 2, -1].
            target_area_max (list, optional): maximum coordinates of the bounding zone. Defaults to [6, 6, 2].
            time (int, optional): time to check the entity (currently not implemented). Defaults to 0.
            tolerance (int, optional): Safety tolerance value that robot shouldn't exceed. Defaults to 0.

        Returns:
            Bool: Returns True if the robot did not go into a forbidden zone.
        """
        all_robot_position = self.primitive_properties.robo_spatial_temporal_information(object)

        result = []

        for robot_position in all_robot_position:

            robot_size_min_x_coord = (-1*self.config.robot_size[0]/2) + tolerance + robot_position[0]
            robot_size_min_y_coord = (-1*self.config.robot_size[1]/2) + tolerance + robot_position[1]
            robot_size_min_z_coord = (-1*self.config.robot_size[2]/2) + tolerance + robot_position[2]

            robot_size_max_x_coord = (self.config.robot_size[0]/2) + tolerance + robot_position[0]
            robot_size_max_y_coord = (self.config.robot_size[1]/2) + tolerance + robot_position[1]        
            robot_size_max_z_coord = (self.config.robot_size[2]/2) + tolerance + robot_position[2]
                
            com_sx =  target_area_min[0]
            com_sy =  target_area_min[1]
            com_sz =  target_area_min[2]

            com_bx =  target_area_max[0]
            com_by =  target_area_max[1]
            com_bz =  target_area_max[2]
            
            # Forbidden Zone
            if (robot_size_min_x_coord < com_bx and robot_size_max_x_coord > com_sx and robot_size_min_y_coord < com_by and robot_size_max_y_coord > com_sy):
                result.append(1)
            else:
                result.append(0)

        robot_at_designated_location = sum(result)
        if robot_at_designated_location > 0:
            return False
        else:
            return True

    def must_be_near_to(self, object='jackal_robot', target_object='convex_5cm', req_dis=6, tolerance=0):
        """Composite property that checks whether the robot is in a given square vicnity.

        Args:
            object (str, optional): The main entity to check the euclidean distance to other object. Defaults to 'jackal_robot'.
            target_object (str, optional): The entity against which the euclidean distance is measured. Defaults to 'convex_5cm'.
            req_dis (int, optional): The minimum distance required between the entities. Defaults to 6.
            tolerance (int, optional): +/- values for required distance. Defaults to 0.

        Returns:
            Bool: Returns True if the main entity is near to the target entity.
        """
        
        all_robot_position = self.primitive_properties.robo_spatial_temporal_information(object)
        target_object = self.primitive_properties.spatial_temporal_information(target_object)
        result = []

        for i in range(len(all_robot_position)):
            dst = distance.euclidean(all_robot_position[i], target_object[i])

            # Need to be within required distance
            if dst < req_dis:
                result.append(0)
            else:
                result.append(1)

        robot_has_less_distance = sum(result)

        if robot_has_less_distance > 0:
            return False
        else:
            return True

    def must_not_be_near_to(self, object='jackal_robot', target_object='convex_5cm', req_dis=6, tolerance=0):
        """Composite property that checks whether the robot is not in a given square vicnity.

        Args:
            object (str, optional): The main entity to check the euclidean distance to other object. Defaults to 'jackal_robot'.
            target_object (str, optional): The entity against which the euclidean distance is measured. Defaults to 'convex_5cm'.
            req_dis (int, optional): The minimum distance required between the entities. Defaults to 6.
            tolerance (int, optional): +/- values for required distance. Defaults to 0.

        Returns:
            Bool: Returns True if the main entity is not near to the target entity.
        """
        all_robot_position = self.primitive_properties.robo_spatial_temporal_information(object)
        target_object = self.primitive_properties.spatial_temporal_information(target_object)
        result = []

        for i in range(len(all_robot_position)):
            dst = distance.euclidean(all_robot_position[i], target_object[i])
            
            # Need to be away required distance
            if dst > req_dis:
                result.append(0)
            else:
                result.append(1)

        robot_has_less_distance = sum(result)

        if robot_has_less_distance > 0:
            return False
        else:
            return True

    def must_have_orientation(self, object='jackal_robot_issac', orientation=[25, 25, 360], time=0, tolerance=0):
        """Composite property that ensures the object is in the correct orientation.

        Args:
            orientation (list, optional): the orientation of the target entity. Defaults to [25, 25, 360].
            time (int, optional): specific time step to check orientation of the entity. Defaults to 0.
            tolerance (int, optional): Extra safety tolerance value that robot shouldn't exceed. Defaults to 0.

        Returns:
            Bool: Returns True if the object orientation was correct.
        """
        object_orientations = self.primitive_properties.spatial_temporal_information(object)
        result = []
        # obj_ori = roll, pitch, yaw
        for obj_ori in object_orientations:
            if obj_ori[0] >= orientation[0] + tolerance:
                result.append(1)
            if obj_ori[1] >= orientation[1] + tolerance:
                result.append(2)
            if obj_ori[2] >= orientation[2] + tolerance:
                result.append(3)

            # print(orientation)
            # print(obj_ori)
        object_at_designated_orientation = sum(result)

        # print(object_at_designated_orientation)

        if object_at_designated_orientation > 0:
            return False
        else:
            return True

    def must_collide(self, robot, object, robot_position=[0,2,0], target_area_min=[-1,-1,0], target_area_max=[2,2,0], time=0, tolerance=0):
        """Composite property that checks for collision of the robot using axis aligned bounding box method.

        Args:
            robot (str): The robot
            object (str): The entity to check against
            robot_position (list, optional): Robots current position. Defaults to [0,2,0].
            target_area_min (list, optional): Entities bounding box size minimum coordinates. Defaults to [-1,-1,0].
            target_area_max (list, optional): Entities bounding box size maximum coordinates. Defaults to [2,2,0].
            time (int, optional): Time to check at. Defaults to 0.
            tolerance (int, optional): Allowable tolerance. Defaults to 0.

        Returns:
            Bool: Returns True if the robot collided with target entity.
        """
        robot_size_min_x_coord = (-1*self.config.robot_size[0]/2) + tolerance + robot_position[0]
        robot_size_min_y_coord = (-1*self.config.robot_size[1]/2) + tolerance + robot_position[1]
        robot_size_min_z_coord = (-1*self.config.robot_size[2]/2) + tolerance + robot_position[2]

        robot_size_max_x_coord = (self.config.robot_size[0]/2) + tolerance + robot_position[0]
        robot_size_max_y_coord = (self.config.robot_size[1]/2) + tolerance + robot_position[1]        
        robot_size_max_z_coord = (self.config.robot_size[2]/2) + tolerance + robot_position[2]
            
        com_sx =  target_area_min[0]
        com_sy =  target_area_min[1]
        com_sz =  target_area_min[2]

        com_bx =  target_area_max[0]
        com_by =  target_area_max[1]
        com_bz =  target_area_max[2]
        
        fig = plt.figure()
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((robot_size_min_x_coord, robot_size_min_y_coord), self.config.robot_size[0], self.config.robot_size[1], fill=False, alpha=1))
        currentAxis.add_patch(Rectangle((com_sx, com_sy), 4, 4, color='red',fill=False, alpha=1))
        plt.show()

        # Collision X-Y-Z-element
        if (robot_size_min_x_coord < com_bx and robot_size_max_x_coord > com_sx and robot_size_min_y_coord < com_by and robot_size_max_y_coord > com_sy and robot_size_min_z_coord < com_bz and robot_size_max_z_coord > com_sz):
            return True
        else:
            return False

    def must_not_collide(self, robot, object):
        """Composite property that checks for collision of the robot using axis aligned bounding box method.

        Args:
            robot (str): The robot
            object (str): The entity to check against
            robot_position (list, optional): Robots current position. Defaults to [0,2,0].
            target_area_min (list, optional): Entities bounding box size minimum coordinates. Defaults to [-1,-1,0].
            target_area_max (list, optional): Entities bounding box size maximum coordinates. Defaults to [2,2,0].
            time (int, optional): Time to check at. Defaults to 0.
            tolerance (int, optional): Allowable tolerance. Defaults to 0.

        Returns:
            Bool: Returns True if the robot did not collide with target entity.
        """
        pass

    def must_have_collision_force_less_than(self, forces_applied:float, threshhold:float):
        """Composite property that checks the collision force of a robot with an entity. It requires a collision sensor.

        Args:
            forces_applied (float): The maximum allowable force.
            threshhold (float): Allowable tolerance.

        Returns:
            Bool: Returns True if the robot collided with target entity.
        """
        result = []
        try:
            for force in forces_applied:
                if abs(force.force.x) >= threshhold: 
                    result.append(1)
                if abs(force.force.y) >= threshhold: 
                    result.append(1)
                if abs(force.force.z) >= threshhold: 
                    result.append(1)

            force_less_than_threshold = sum(result)
            if force_less_than_threshold > 0:
                return False
            else:
                return True
        except:
            return False

    # Callback functions
    def robot_force_sensor_callback(self, data):
        info = data.states
        if info:
            self.collider_1 = info[0].collision1_name
            self.collider_2 = info[0].collision2_name
            self.collision_force = info[0].wrenches
            self.in_collision = True   


if __name__ == '__main__':
    # u = CompositeProperties()
    # print(u.must_be_at())
    # print(u.must_be_near_to())
    # print(u.must_not_be_at())
    # print(u.must_have_orientation(object='pay_load_square', orientation=[15, 15, 360], time=0, tolerance=1))
    # print(u.must_have_orientation(object='husky', orientation=[6, 6, 360], time=0, tolerance=1))
    pass