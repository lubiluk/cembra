#!/usr/bin/env python

import rospy
import sensor_msgs.msg
from cembra.srv import Action, ActionResponse
from tmc_msgs.msg import JointVelocity
from geometry_msgs.msg import Twist
import control_msgs.msg
import controller_manager_msgs.srv
from PIL import Image
from prep import prepare_gripper, prepare_head

CAMERA_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
VELOCITY_TOPIC = '/cembra/velocity'
TWIST_TOPIC = '/cembra/twist'
SLOW = 0.1
STOP = 0.0

vel_pub = None
twist_pub = None

def get_image():
    msg = rospy.wait_for_message(CAMERA_TOPIC, sensor_msgs.msg.Image)
    img = Image.frombytes('RGB', (msg.width, msg.height), msg.data)
    img.save('camera.jpg')
    return msg

def handle_action(msg):
    actions = msg.action
    
    handle_velocity_actions(actions[:-1])
    handle_twist_action(actions[-1])

    # Let things happen for a while
    rospy.sleep(0.02)

    img = get_image()

    response = ActionResponse()
    response.state = img
    response.reward = 0

    return response

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
    return action * SLOW

def run():
    global vel_pub
    global twist_pub

    rospy.init_node('environment')

    prepare_head()
    prepare_gripper()

    # initialize ROS publisher
    # queue size of one will keep only the newest message
    vel_pub = rospy.Publisher(VELOCITY_TOPIC, JointVelocity, queue_size=1)
    twist_pub = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=1)

    rospy.loginfo("Waiting for filters to attach")
    # wait to establish connection between the controller
    while vel_pub.get_num_connections() == 0 or twist_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    s = rospy.Service('/cembra/action', Action, handle_action)
    rospy.loginfo("Environment ready")
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass