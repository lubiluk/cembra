#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import tmc_msgs.msg

rospy.init_node('random_motion')

# initialize ROS publisher
pub = rospy.Publisher('/hsrb/pseudo_velocity_controller/ref_joint_velocity',
                      tmc_msgs.msg.JointVelocity, queue_size=1)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = (
    rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                       controller_manager_msgs.srv.ListControllers))
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'joint_state_controller' and c.state == 'running':
            running = True

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    # fill ROS message
    vel = tmc_msgs.msg.JointVelocity()
    vel.header.stamp = rospy.Time.now()
    vel.name = ["wrist_flex_joint"]
    vel.velocity = [ -0.75 ]

    # publish ROS message
    pub.publish(vel)

    # sleep
    rate.sleep()