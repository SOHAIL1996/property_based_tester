#!/usr/bin/env python
"""
---------------------------------------------------- 
Movement client

The client activates the MDR navigation module.
----------------------------------------------------
Supervisor: Prof. Dr. Nico Hochgeschwender
            Prof. Dr. Paul Ploger
            Sven Schneider 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: July 01, 2022
----------------------------------------------------
"""
import time
import rospy
from geometry_msgs.msg import Twist

def move(speed=1, cmd_vel_topic='/cmd_vel', timeout=5):

    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    start = time.time()
    time_elapsed = 0

    while(time_elapsed <= timeout):

        velocity_publisher.publish(vel_msg)
        
        # Time duration to move
        end = time.time()
        time_elapsed = end - start

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    return True
