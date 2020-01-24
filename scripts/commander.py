#!/usr/bin/env python

import rospy
import tmc_msgs.msg
import random

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

def run():
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