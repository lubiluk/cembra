#!/usr/bin/env python

import rospy
from velocity_generator import VelocityGenerator
from velocity_validator import VelocityValidator
import geometry_msgs.msg
import tmc_msgs.msg
import sensor_msgs.msg
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg


rospy.init_node('random_motion')
        
def prepare_gripper():
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
    running = False
    while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'gripper_controller' and c.state == 'running':
                running = True

    # fill ROS message
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["hand_motor_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [0]
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
    prepare_gripper()

    gen = VelocityGenerator()
    val = VelocityValidator()

    def joint_state_callback(msg):
        val.update_joint_states(msg)

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
    gen.regen_base_twist()
    gen.regen_arm_velocities()

    def timeout_callback(event):
        gen.regen_arm_velocities()
        gen.regen_base_twist()

    rospy.Timer(rospy.Duration(5), timeout_callback)

    while not rospy.is_shutdown():
        # fill ROS message
        (vel, twist) = gen.get_velocities()
        vel_ok = val.is_joint_vel_ok(vel)
        twist_ok = val.is_base_twist_ok(twist)

        # publish ROS message
        if vel_ok:
            vel_pub.publish(vel)
        else:
            vel_pub.publish(gen.get_arm_stop_velocities())
            gen.regen_arm_velocities()
        
        if twist_ok:
            twist_pub.publish(twist)
            pass
        else:
            twist_pub.publish(gen.get_stop_base_twist())
            gen.regen_base_twist()
        
        # sleep
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
