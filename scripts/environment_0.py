#!/usr/bin/env python

import math as m
import random

import control_msgs.msg
import controller_manager_msgs.srv
import gazebo_msgs.msg
import gazebo_msgs.srv
import geometry_msgs.msg
import rospy
import sensor_msgs.msg
import tmc_msgs.msg
from PIL import Image

import cembra.srv
from prep import prepare_gripper, prepare_head

CAMERA_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
VELOCITY_TOPIC = '/cembra/velocity'
TWIST_TOPIC = '/cembra/twist'
SLOW = 0.1
STOP = 0.0
CUBE_Z = -0.001001
CAN_Z = -0.02621
MIN_ROBOT_DISTANCE = 0.5
MAX_ROBOT_DISTANCE = 0.9
MIN_ROBOT_ANGLE = -m.pi / 2
MAX_ROBOT_ANGLE = m.pi / 2
CAN_DISTANCE = 0.15
MIN_CUBE_CAN_DISTANCE = 0.1

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

    response = cembra.srv.ActionResponse()
    response.state = img
    response.reward = 0
    response.is_done = 0

    return response

def handle_velocity_actions(actions):
    assert(len(actions) == 5)

    msg = tmc_msgs.msg.JointVelocity()
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
    msg = geometry_msgs.msg.Twist()
    msg.angular.z = action_to_velocity(action)
    
    twist_pub.publish(msg)

def action_to_velocity(action):
    return action * SLOW

def handle_reset(action):
    # Choose location of the first can
    r1 = random.uniform(MIN_ROBOT_DISTANCE, MAX_ROBOT_DISTANCE)
    theta1 = random.uniform(MIN_ROBOT_ANGLE, MAX_ROBOT_ANGLE)
    x1 = r1 * m.cos(theta1)
    y1 = r1 * m.sin(theta1)

    # Choose location of the second can
    ok = False
    while not ok:
        thetac = random.uniform(0, 2 * m.pi)
        xc = CAN_DISTANCE * m.cos(thetac)
        yc = CAN_DISTANCE * m.sin(thetac)

        x2 = x1 + xc
        y2 = y1 + yc
        r2 = m.sqrt(x2**2 + y2**2)
        theta2 = m.atan(y2 / x2)

        ok = MIN_ROBOT_ANGLE < theta2 < MAX_ROBOT_ANGLE and MIN_ROBOT_DISTANCE < r2 < MAX_ROBOT_DISTANCE

    # Choose location of the cube
    ok = False
    while not ok:
        r3 = random.uniform(MIN_ROBOT_DISTANCE, MAX_ROBOT_DISTANCE)
        theta3 = random.uniform(MIN_ROBOT_ANGLE, MAX_ROBOT_ANGLE)
        x3 = r3 * m.cos(theta3)
        y3 = r3 * m.sin(theta3)

        distance1 = m.sqrt((x1 - x3)**2 + (y1 - y3)**2)
        distance2 = m.sqrt((x2 - x3)**2 + (y2 - y3)**2)

        ok =  distance1 > MIN_CUBE_CAN_DISTANCE and distance2 > MIN_CUBE_CAN_DISTANCE

    # Set object positions
    try:
        # Set coke_can1 state
        state = gazebo_msgs.msg.ModelState()
        state.pose.position.x = x1
        state.pose.position.y = y1
        state.reference_frame = "world"
        state.model_name = "coke_can1"

        perform_action = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        response = perform_action(state)

        # Set coke_can2 state
        state = gazebo_msgs.msg.ModelState()
        state.pose.position.x = x2
        state.pose.position.y = y2
        state.reference_frame = "world"
        state.model_name = "coke_can2"

        perform_action = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        response = perform_action(state)

        # Set cube state
        state = gazebo_msgs.msg.ModelState()
        state.pose.position.x = x3
        state.pose.position.y = y3
        state.reference_frame = "world"
        state.model_name = "cube"

        perform_action = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        response = perform_action(state)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

    rospy.sleep(1)

    img = get_image()

    response = cembra.srv.ResetResponse()
    response.state = img
    response.reward = 0
    response.isdone = 0

    return response

def run():
    global vel_pub
    global twist_pub

    rospy.init_node('environment')

    prepare_head()
    prepare_gripper()

    # initialize ROS publisher
    # queue size of one will keep only the newest message
    vel_pub = rospy.Publisher(VELOCITY_TOPIC, tmc_msgs.msg.JointVelocity, queue_size=1)
    twist_pub = rospy.Publisher(TWIST_TOPIC, geometry_msgs.msg.Twist, queue_size=1)

    rospy.loginfo("Waiting for filters to attach")
    # wait to establish connection between the controller
    while vel_pub.get_num_connections() == 0 or twist_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    sr = rospy.Service('/cembra/reset', cembra.srv.Reset, handle_reset)
    sa = rospy.Service('/cembra/action', cembra.srv.Action, handle_action)
    rospy.loginfo("Environment ready")
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
