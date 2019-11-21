import math

MAX_VELOCITY = 1
MAX_ROTATION = math.pi / 2
VEL = -1
MIN_PALM_HEIGHT = 0.04
MIN_GRIPPER_HEIGHT = 0.04
MAX_PALM_HEIGHT = 0.5
MAX_GRIPPER_HEIGHT = 0.5
REFERENCE_FRAME = 'base_footprint'

GRIPPER_LINKS = [
    'hand_l_distal_link',
    'hand_l_finger_tip_frame',
    'hand_l_finger_vacuum_frame',
    'hand_l_mimic_distal_link',
    'hand_l_proximal_link',
    'hand_l_spring_proximal_link',
    'hand_r_distal_link',
    'hand_r_finger_tip_frame',
    'hand_r_mimic_distal_link',
    'hand_r_proximal_link',
    'hand_r_spring_proximal_link',
]

PALM_LINKS = [
    'hand_palm_link',
    'hand_camera_frame'
]