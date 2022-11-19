#!/usr/bin/env python
"""
Movement Client: The client activates the provides raw command velocity to mobile
robots.
"""
import time
import rospy
from geometry_msgs.msg import Twist

def move(speed=1, cmd_vel_topic='/cmd_vel', timeout=5):
    """Utilizes a ROS publiser to provide raw command velocities in a given
    topic to move a mobile robot.

    Args:
        speed (int, optional): The value of speed that the robot should move with. Defaults to 1.
        cmd_vel_topic (str, optional): The receiving topic by which the robot moves. Defaults to '/cmd_vel'.
        timeout (int, optional): The amount of time to publish. Defaults to 5.

    Returns:
        bool: Returns True if publishing was successful.
    """

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