#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import tmc_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf
import math

MAX_VELOCITY = 1
MAX_ROTATION = math.pi / 2
VEL = -1
MIN_PALM_HEIGHT = 0.03
MIN_GRIPPER_HEIGHT = 0.02

GRIPPER_LINKS = [
    'hand_l_distal_link',
    'hand_l_finger_tip_frame',
    'hand_l_finger_vacuum_frame',
    'hand_l_mimic_distal_link',
    'hand_l_proximal_link',
    'hand_l_spring_proximal_link',
    'hand_r_distal_link',
    'hand_r_finger_tip_frame',
    'hand_r_mimic_distal_link',
    'hand_r_proximal_link',
    'hand_r_spring_proximal_link',
]

PALM_LINKS = [
    'hand_palm_link',
    'hand_camera_frame'
]

rospy.init_node('random_motion')

class VelocityGenerator:
    def get_velocities(self):
        joint_vel = tmc_msgs.msg.JointVelocity()
        joint_vel.name = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
            ]

        joint_vel.velocity = [
            VEL,
            VEL,
            VEL,
            VEL,
            VEL
        ]
        
        joint_vel.header.stamp = rospy.Time.now()

        base_twist = geometry_msgs.msg.Twist()
        base_twist.angular.z = VEL

        return (joint_vel, base_twist)


class MotionConstrainer:
    initial_rotation = None
    listener = None

    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.initial_rotation = self.get_current_rotation()

    def constrain_joint_vel(self, joint_vel):
        # for link in PALM_LINKS:
        #     ts = self.buffer.lookup_transform('base_footprint', link, rospy.Time())
            
        #     if ts.transform.translation.z <= MIN_PALM_HEIGHT:


        return joint_vel

    def constrain_base_twist(self, base_twist):
        c = math.pi
        x = self.initial_rotation
        y = self.get_current_rotation()

        diff = c - abs(abs(x - y) % (2 * c) - c)

        if diff >= MAX_ROTATION:
            base_twist.angular.z = 0

        return base_twist

    def get_current_rotation(self):
        trans = self.buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(10))
        quat = trans.transform.rotation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

        

def run():
    # initialize ROS publisher
    vel_pub = rospy.Publisher('/hsrb/pseudo_velocity_controller/ref_joint_velocity',
                        tmc_msgs.msg.JointVelocity, queue_size=1)
    twist_pub = rospy.Publisher('/hsrb/command_velocity',
                        geometry_msgs.msg.Twist, queue_size=1)
    # queue size of one will keep only the newest message

    # wait to establish connection between the controller
    while vel_pub.get_num_connections() == 0 or twist_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    from urdf_parser_py.urdf import URDF
    from pykdl_utils.kdl_kinematics import KDLKinematics
    robot = URDF.from_parameter_server()
    kdl_kin = KDLKinematics(robot, 'base_footprint', 'hand_palm_link')
    pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)

    print 'pose:', pose

    exit()


    # Send messages with 100 hz rate
    rate = rospy.Rate(100)
    gen = VelocityGenerator()
    con = MotionConstrainer()

    while not rospy.is_shutdown():
        # fill ROS message
        (vel, twist) = gen.get_velocities()
        con_vel = con.constrain_joint_vel(vel)
        con_twist = con.constrain_base_twist(twist)

        # publish ROS message
        vel_pub.publish(con_vel)
        twist_pub.publish(con_twist)

        # sleep
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
