#!/usr/bin/env python2
"""
Pick and place sample by Yoshimi_yoda: Copyright (C) 2017 Toyota Motor Corporation
"""
from copy import deepcopy
import math
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg


class MoveItPickAndPlace(object):
    """Using built in moveit commander for pick and place tasks for the Toyota HSR.

    Args:
        object (class): Inherited class from moveit commander
    """
    def __init__(self, pick_x = 0, pick_y = 0, pick_z = 0, place_x = 0, place_y = 0, place_z = 0,wait=0.0):

        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("moveit_demo", anonymous=True)

        self.reference_frame = "odom"
        arm = moveit_commander.MoveGroupCommander("arm")
        base = moveit_commander.MoveGroupCommander("base")
        gripper = moveit_commander.MoveGroupCommander("gripper")
        head = moveit_commander.MoveGroupCommander("head")
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body.allow_replanning(True)
        self.whole_body.set_planning_time(5)
        self.whole_body.set_pose_reference_frame(self.reference_frame)
        self.end_effector = self.whole_body.get_end_effector_link()
        self.pick_x = pick_x
        self.pick_y = pick_y
        self.pick_z = pick_z
        self.place_x = place_x
        self.place_y = place_y
        self.place_z = place_z
        rospy.sleep(1)
        ##############################################################################
        # remove all objects
        self.scene.remove_attached_object(self.end_effector)
        self.scene.remove_world_object()
        rospy.sleep(1)
        ##############################################################################
        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")
        # base.go()
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        gripper.set_joint_value_target("hand_motor_joint", 0.75)
        gripper.go()
        rospy.logdebug("done")
        rospy.sleep(wait)
        ##############################################################################
        # add objects
        # top 3 are box coordinates bottom 3 are pose coordinates
        self.add_box("table",
                     [0.63, 0.63, 0.48],
                     [self.pick_x, self.pick_y, 0.25])
        # self.add_box("wall",
        #              [0.3, 0.01, 0.1],
        #              [0.5, 0.0, 0.5 + 0.1 / 2])
        self.add_box("target1",
                     [0.01, 0.01, 0.1],
                     [self.pick_x, self.pick_y, self.pick_z])
        ##############################################################################
        # pick target1
        self.whole_body.set_support_surface_name("table")
        rospy.loginfo("step2: pick target1")
        grasps = self.make_grasps("target1",
                                  (0.707, 0.0, 0.707, 0.0),
                                  quality=lambda x, y, z, roll, pitch, yaw: 1 - abs(pitch),  # noqa
                                  x=[-0.07],
                                  pitch=[-0.2, -0.1, 0, 0.1, 0.2])
        self.pick("target1", grasps)
        rospy.logdebug("done")
        rospy.sleep(wait)
                
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()

    def pick(self, target, grasps):

        n_attempts = 0
        max_pick_attempts = 10
        result = None

        while (result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS) and \
              (n_attempts < max_pick_attempts):
            n_attempts += 1
            rospy.loginfo("Pick attempt: " + str(n_attempts))
            result = self.whole_body.pick(target, grasps)
            rospy.sleep(0.2)
        if result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            self.scene.remove_attached_object(self.end_effector)
        return result

    def place(self, target, location):
        n_attempts = 0
        max_pick_attempts = 10
        result = None

        while (result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS) and \
              (n_attempts < max_pick_attempts):
            n_attempts += 1
            rospy.loginfo("Place attempt: " + str(n_attempts))
            result = self.whole_body.place(target, location)
            rospy.sleep(0.2)
        if result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            self.scene.remove_attached_object(self.end_effector)
        return result

    def make_gripper_posture(self, pos, effort=0.0):
        t = trajectory_msgs.msg.JointTrajectory()
        t.joint_names = ["hand_motor_joint"]
        tp = trajectory_msgs.msg.JointTrajectoryPoint()
        tp.positions = [pos]
        tp.effort = [effort]
        tp.time_from_start = rospy.Duration(2.0)
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, vector, frame=None):
        g = moveit_msgs.msg.GripperTranslation()
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        if frame is None:
            g.direction.header.frame_id = self.end_effector
        else:
            g.direction.header.frame_id = frame
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_pose(self, init, x, y, z, roll, pitch, yaw):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.reference_frame
        q = quaternion_from_euler(roll, pitch, yaw)
        q = quaternion_multiply(init, q)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    def make_grasps(self, target, init,
                    quality=None,
                    x=[0], y=[0], z=[0],
                    roll=[0], pitch=[0], yaw=[0]):
        poses = self.scene.get_object_poses([target])
        pose = poses[target]
        g = moveit_msgs.msg.Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(0.8)
        g.grasp_posture = self.make_gripper_posture(0.2, -0.01)
        g.pre_grasp_approach \
            = self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0])
        g.post_grasp_retreat \
            = self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0],
                                            "base_footprint")
        grasps = []
        for ix in x:
            for iy in y:
                for iz in z:
                    for iroll in roll:
                        for ipitch in pitch:
                            for iyaw in yaw:
                                x = pose.position.x + ix
                                y = pose.position.y + iy
                                z = pose.position.z + iz
                                g.grasp_pose = self.make_pose(init,
                                                              x, y, z,
                                                              iroll,
                                                              ipitch,
                                                              iyaw)
            g.id = str(len(grasps))
            g.allowed_touch_objects = ["target1"]
            g.max_contact_force = 0
            if quality is None:
                g.grasp_quality = 1.0
            else:
                g.grasp_quality = quality(ix, iy, iz, iroll, ipitch, iyaw)
            grasps.append(deepcopy(g))
        return grasps

    def make_place_location(self, x, y, z):
        location = moveit_msgs.msg.PlaceLocation()
        location.pre_place_approach \
            = self.make_gripper_translation(0.03, 0.05, [0, 0, -1.0],
                                            "base_footprint")
        location.post_place_posture \
            = self.make_gripper_posture(0.8)
        location.post_place_retreat \
            = self.make_gripper_translation(0.03, 0.05, [0, 0, -1.0])
        location.place_pose = self.make_pose((0, 0, 0, 1),
                                             x, y, z,
                                             0, 0, 0)
        return location

    def add_box(self, name, size, pos):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.reference_frame
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.w = 1.0
        self.scene.add_box(name, p, size)

    def add_cylinder(self, name, radius, height, pos):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header.frame_id = self.reference_frame
        box = shape_msgs.msg.SolidPrimitive()
        box.type = shape_msgs.msg.SolidPrimitive.CYLINDER
        box.dimensions = [height, radius]
        co.primitives = [box]
        p = geometry_msgs.msg.Pose()
        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]
        co.primitive_poses = [p]
        self.scene._pub_co.publish(co)