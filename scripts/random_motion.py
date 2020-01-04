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
import tf2_ros
import tf

rospy.init_node('random_motion')


def prepare_gripper():
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    print("Waiting for trajectory action")
    cli.wait_for_server()

    print("Waiting for controller")
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

    print("Preparing gripper")

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
    # setup gripper to desired position
    # prepare_gripper() # gripper is not available in simulation

    # TF
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # create generator and validator
    gen = VelocityGenerator()
    val = VelocityValidator()

    def joint_state_callback(msg):
        val.current_joint_states = msg

    def get_current_rotation():
        trans = buffer.lookup_transform(
            'map', 'base_link', rospy.Time(), rospy.Duration(10))
        quat = trans.transform.rotation
        euler = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    # stup reference rotation for rotation validation
    val.reference_rotation = get_current_rotation()

    # initialize ROS publisher
    vel_pub = rospy.Publisher('/hsrb/pseudo_velocity_controller/ref_joint_velocity',
                              tmc_msgs.msg.JointVelocity, queue_size=1)
    twist_pub = rospy.Publisher('/hsrb/command_velocity',
                                geometry_msgs.msg.Twist, queue_size=1)
    rospy.Subscriber('/hsrb/robot_state/joint_states',
                     sensor_msgs.msg.JointState, joint_state_callback, queue_size=1)
    # queue size of one will keep only the newest message

    # wait to establish connection between the controller
    while vel_pub.get_num_connections() == 0 or twist_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # Send messages with 100 hz rate
    rate = rospy.Rate(100)

    # generate velocities
    gen.regen_base_twist()
    gen.regen_arm_velocities()

    def timeout_callback(event):
        print("regenerating velocities")
        gen.regen_arm_velocities()
        gen.regen_base_twist()

    # change velocities periodically so it looks more interesting
    rospy.Timer(rospy.Duration(5), timeout_callback)

    while not rospy.is_shutdown():
        val.current_rotation = get_current_rotation()
        (vel, twist) = gen.get_velocities()
        val.requested_velocities = vel
        val.requested_twist = twist

        # publish ROS message
        if val.velocities_ok:
            vel_pub.publish(vel)
        else:
            vel_pub.publish(gen.get_arm_stop_velocities())
            gen.regen_arm_velocities()

        if val.twist_ok:
            twist_pub.publish(twist)
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
