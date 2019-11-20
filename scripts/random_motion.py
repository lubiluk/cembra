#!/usr/bin/env python

import controller_manager_msgs.srv
import rospy
import tmc_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf
import math
import sensor_msgs.msg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

MAX_VELOCITY = 1
MAX_ROTATION = math.pi / 2
VEL = -1
MIN_PALM_HEIGHT = 0.06
MIN_GRIPPER_HEIGHT = 0.06
REFERENCE_FRAME = 'base_footprint'

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
            0,
            0,
            0,
            0
        ]
        
        joint_vel.header.stamp = rospy.Time.now()

        base_twist = geometry_msgs.msg.Twist()
        base_twist.angular.z = VEL

        return (joint_vel, base_twist)


class MotionConstrainer:
    initial_rotation = None
    listener = None
    joint_states = None
    robot = None

    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.initial_rotation = self.get_current_rotation()
        self.robot = URDF.from_parameter_server()

    def is_joint_vel_ok(self, joint_vel):
        is_violating = True

        for link in PALM_LINKS:
            kinematics = KDLKinematics(self.robot, REFERENCE_FRAME, link)
            angles = [self.joint_states.position[self.joint_states.name.index(name)] for name in kinematics.get_joint_names()]
            pose = kinematics.forward(angles)
            z = pose[2,3]
            
            if z <= MIN_PALM_HEIGHT:
                print("Palm Cutout: {} ({})".format(link, z))
                is_violating = False

        for link in GRIPPER_LINKS:
            kinematics = KDLKinematics(self.robot, REFERENCE_FRAME, link)
            angles = [self.joint_states.position[self.joint_states.name.index(name)] for name in kinematics.get_joint_names()]
            pose = kinematics.forward(angles)
            z = pose[2,3]
            
            if z <= MIN_GRIPPER_HEIGHT:
                print("Gripper Cutout: {} ({})".format(link, z))
                is_violating = False

        return is_violating

    def is_base_twist_ok(self, base_twist):
        c = math.pi
        x = self.initial_rotation
        y = self.get_current_rotation()

        diff = c - abs(abs(x - y) % (2 * c) - c)

        if diff >= MAX_ROTATION:
            print("Base Cutout ({})".format(diff))
            return False

        return True

    def get_current_rotation(self):
        trans = self.buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(10))
        quat = trans.transform.rotation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def update_joint_states(self, msg):
        self.joint_states = msg
        

def run():
    gen = VelocityGenerator()
    con = MotionConstrainer()

    def joint_state_callback(msg):
        con.update_joint_states(msg)

    # initialize ROS publisher
    vel_pub = rospy.Publisher('/hsrb/pseudo_velocity_controller/ref_joint_velocity',
                        tmc_msgs.msg.JointVelocity, queue_size=1)
    twist_pub = rospy.Publisher('/hsrb/command_velocity',
                        geometry_msgs.msg.Twist, queue_size=1)
    rospy.Subscriber('/hsrb/robot_state/joint_states', sensor_msgs.msg.JointState, joint_state_callback, queue_size=1)
    # queue size of one will keep only the newest message

    # wait to establish connection between the controller
    while vel_pub.get_num_connections() == 0 or twist_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # Send messages with 100 hz rate
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # fill ROS message
        (vel, twist) = gen.get_velocities()
        vel_ok = con.is_joint_vel_ok(vel)
        twist_ok = con.is_base_twist_ok(twist)

        # publish ROS message
        if vel_ok:
            vel_pub.publish(vel)
        
        # if twist_ok:
        #     twist_pub.publish(twist)

        # sleep
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
