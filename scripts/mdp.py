#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cembra.srv import Action, ActionResponse
from tmc_msgs.msg import JointVelocity
from geometry_msgs.msg import Twist
import trajectory_msgs.msg
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv

CAMERA_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
VELOCITY_TOPIC = '/cembra/velocity'
TWIST_TOPIC = '/cembra/twist'
FAST = 0.3
SLOW = 0.1
STOP = 0.0

vel_pub = None
twist_pub = None

def get_image():
    msg = rospy.wait_for_message(CAMERA_TOPIC, Image)
    print(msg)

def handle_action(msg):
    actions = msg.action
    
    handle_velocity_actions(actions[:-1])
    handle_twist_action(actions[-1])

    # Let things happen for a while
    rospy.sleep(0.02)

    return ActionResponse(0)

def handle_velocity_actions(actions):
    assert(len(actions) == 5)

    msg = JointVelocity()
    msg.name = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
            ]
    msg.velocity = [action_to_velocity(a) for a in actions]
    msg.header.stamp = rospy.Time.now()

    vel_pub.publish(msg)

def handle_twist_action(action):
    msg = Twist()
    msg.angular.z = action_to_velocity(action)
    
    twist_pub.publish(msg)

def action_to_velocity(action):
    return {
        0: STOP,
        1: SLOW,
        2: FAST
    }[action]

def prepare_head():
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/head_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # fill ROS message
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [0., -0.7]
    p.velocities = [0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]
    goal.trajectory = traj

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()
    
def prepare_gripper():
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # fill ROS message
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["hand_motor_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [0.0]
    p.velocities = [0]
    p.effort = [0.1]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]
    goal.trajectory = traj

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

def run():
    global vel_pub
    global twist_pub

    rospy.init_node('mdp')

    prepare_head()
    prepare_gripper()

    # initialize ROS publisher
    # queue size of one will keep only the newest message
    vel_pub = rospy.Publisher(VELOCITY_TOPIC, JointVelocity, queue_size=1)
    twist_pub = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=1)

    # wait to establish connection between the controller
    while vel_pub.get_num_connections() == 0 or twist_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    s = rospy.Service('action', Action, handle_action)
    print("MDP ready")
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass