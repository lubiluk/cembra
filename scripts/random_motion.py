#!/usr/bin/env python

import rospy
from velocity_generator import VelocityGenerator
from velocity_validator import VelocityValidator
import geometry_msgs.msg
import tmc_msgs.msg
import sensor_msgs.msg

rospy.init_node('random_motion')
        

def run():
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
