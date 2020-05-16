
import math as m

FILTERED_VELOCITY_TOPIC = '/cembra/velocity'
UNFILTERED_VELOCITY_TOPIC = '/hsrb/pseudo_velocity_controller/ref_joint_velocity'
FILTERED_TWIST_TOPIC = '/cembra/twist'
UNFILTERED_TWIST_TOPIC = '/hsrb/command_velocity'
RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'
SLOW = 0.1
STOP = 0.0
CUBE_Z = -0.001001
CAMERA_HEIGHT = 0.8
MIN_ROBOT_DISTANCE = 0.5
MAX_ROBOT_DISTANCE = 0.9
MIN_ROBOT_ANGLE = -0.30
MAX_ROBOT_ANGLE = 0.30
START_ARM_JOINT_NAMES = [
    'arm_flex_joint', 
    'arm_lift_joint', 
    'arm_roll_joint', 
    'wrist_flex_joint', 
    'wrist_roll_joint'
]
START_ARM_JOINT_ANGLES = [
    -0.023208623448968346, 
    -2.497211361823794e-06, 
    1.57, 
    -1.57, 
    -0.0008994942881326295
]
START_GRIPPR_JOINT_ANGLE = 0
START_GRIPPR_JOINT_EFFORT = 0.1
START_HEAD_PAN = 0
START_HEAD_TILT = -0.9