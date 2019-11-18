#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import tmc_msgs.msg

# Control is split into three different topics for each robot part:
# Base, Arm, Head
# The reason is that it is not possible to 

MIN_VEL = -1
MAX_VEL = 1

rospy.init_node('random_motion')

class VelocityGenerator:
    last_change = None
    velocities = None

    def get_velocities(self):
        self.generate_velocities()
        self.velocities.header.stamp = rospy.Time.now()
        
        return self.velocities

    def generate_velocities(self):
        self.velocities = tmc_msgs.msg.JointVelocity()
        self.velocities.name = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
            ]

        self.velocities.velocity = [
            1,
            1,
            1,
            1,
            1
        ]

        self.last_change = rospy.Time.now()

        

def run():
    # initialize ROS publisher
    pub = rospy.Publisher('/hsrb/pseudo_velocity_controller/ref_joint_velocity',
                        tmc_msgs.msg.JointVelocity, queue_size=1)
    # queue size of one will keep only the newest message

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

    # Send messages with 100 hz rate
    rate = rospy.Rate(100)
    gen = VelocityGenerator()

    while not rospy.is_shutdown():
        # fill ROS message
        vel = gen.get_velocities()

        # publish ROS message
        pub.publish(vel)

        # sleep
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
