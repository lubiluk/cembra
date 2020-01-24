#!/usr/bin/env python

import rospy
import tmc_msgs.msg
import random
import trajectory_msgs.msg
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv

rospy.init_node('commander')

def get_velocity():
    vel = tmc_msgs.msg.JointVelocity()

    vel.name = [
        'arm_flex_joint',
        'arm_lift_joint',
        'arm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint'
    ]

    vel.velocity = [
        -1,
        0,
        0,
        0,
        0
    ]
            
    # vel.velocity = [
    #     random.uniform(-0.5, 0.5),
    #     random.uniform(-0.5, 0.5),
    #     random.uniform(-0.5, 0.5),
    #     random.uniform(-0.5, 0.5),
    #     random.uniform(-0.5, 0.5)
    # ]

    return vel

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
    prepare_head()
    prepare_gripper()

    # initialize ROS publisher
    vel_pub = rospy.Publisher('/cembra/velocity',
                              tmc_msgs.msg.JointVelocity, queue_size=1)

    rate = rospy.Rate(100)
    
    vel = get_velocity()

    while not rospy.is_shutdown():
        vel.header.stamp = rospy.Time.now()
        vel_pub.publish(vel)

        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass