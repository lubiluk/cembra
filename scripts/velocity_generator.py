import rospy
import tmc_msgs.msg
import geometry_msgs.msg
from constants import *
import random

class VelocityGenerator(object):
    base_twist = geometry_msgs.msg.Twist()
    joint_vel = tmc_msgs.msg.JointVelocity()
    stop_base_twist = geometry_msgs.msg.Twist()
    stop_joint_velocities = tmc_msgs.msg.JointVelocity()

    def __init__(self):
        self.stop_base_twist.angular.z = 0
        self.joint_vel.name = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
            ]
        self.stop_joint_velocities.name = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
            ]
        self.stop_joint_velocities.velocity = [ 0, 0, 0, 0, 0 ]

    def regen_base_twist(self):
        self.base_twist.angular.z = 0

    def regen_arm_velocities(self):
        self.joint_vel.velocity = [
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5)
        ]


    def get_velocities(self):
        self.joint_vel.header.stamp = rospy.Time.now()
        return (self.joint_vel, self.base_twist)

    def get_arm_stop_velocities(self):
        self.stop_joint_velocities.header.stamp = rospy.Time.now()
        return self.stop_joint_velocities

    def get_stop_base_twist(self):
        return self.stop_base_twist
        