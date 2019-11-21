
import tf2_ros
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from constants import *
import rospy
import tf
import copy

class VelocityValidator:
    initial_rotation = None
    listener = None
    joint_states = None
    robot = None
    kinematics = {}

    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.initial_rotation = self.get_current_rotation()
        self.robot = URDF.from_parameter_server()

        for link in PALM_LINKS:
            self.kinematics[link] = KDLKinematics(self.robot, REFERENCE_FRAME, link)
        for link in GRIPPER_LINKS:
            self.kinematics[link] = KDLKinematics(self.robot, REFERENCE_FRAME, link)

    def is_link_trajectory_ok(self, link, predicted_joint_states, lower_bound, upper_bound):
        kinematics = self.kinematics[link]
            
        (current_angles, _, _) = kinematics.extract_joint_state(self.joint_states)
        current_pose = kinematics.forward(current_angles)
        current_height = current_pose[2,3]

        (predicted_angles, _, _) = kinematics.extract_joint_state(predicted_joint_states)
        predicted_pose = kinematics.forward(predicted_angles)
        predicted_height = predicted_pose[2,3]

        if predicted_height <= lower_bound and predicted_height <= current_height:
            print("arm lower bound violation: {} ({} => {})".format(link, current_height, predicted_height))
            return False

        if predicted_height >= upper_bound and predicted_height >= current_height:
            print("arm upper bound violation: {} ({} => {})".format(link, current_height, predicted_height))
            return False

        return True

    def is_joint_vel_ok(self, joint_velocities):
        predicted_joint_states = copy.deepcopy(self.joint_states)
        predicted_joint_positions = list(self.joint_states.position)
        predicted_joint_velocities = list(self.joint_states.velocity)

        for i, n in enumerate(joint_velocities.name):
            v = joint_velocities.velocity[i]
            index = predicted_joint_states.name.index(n)
            # desired angular velocity assuming 1 second drift
            predicted_joint_positions[index] += 0.02 if v >= 0 else -0.02
            predicted_joint_velocities[index] = v

        predicted_joint_states.position = tuple(predicted_joint_positions)
        predicted_joint_states.velocity = tuple(predicted_joint_velocities)

        for link in PALM_LINKS:
            if not self.is_link_trajectory_ok(link, predicted_joint_states, MIN_PALM_HEIGHT, MAX_PALM_HEIGHT):
                return False

        for link in GRIPPER_LINKS:
            if not self.is_link_trajectory_ok(link, predicted_joint_states, MIN_GRIPPER_HEIGHT, MAX_GRIPPER_HEIGHT):
                return False

        # kinematics = self.kinematics["hand_palm_link"]
        # (current_angles, _, _) = kinematics.extract_joint_state(self.joint_states)
        # current_pose = kinematics.forward(current_angles)
        # current_height = current_pose[2,3]

        # (predicted_angles, _, _) = kinematics.extract_joint_state(predicted_joint_states)
        # predicted_pose = kinematics.forward(predicted_angles)
        # predicted_height = predicted_pose[2,3]

        return True

    def is_base_twist_ok(self, base_twist):
        sign = 1 if base_twist.angular.z >= 0 else -1
        base = math.pi
        initial_rotation = self.initial_rotation
        current_rotation = self.get_current_rotation()
        predicted_rotation = self.get_current_rotation() + sign * 0.5

        current_diff = base - abs(abs(initial_rotation - current_rotation) % (2 * base) - base)
        predicted_diff = base - abs(abs(initial_rotation - predicted_rotation) % (2 * base) - base)
        
        if predicted_diff >= MAX_ROTATION and predicted_diff >= current_diff:
            print("base violation ({} => {})".format(current_diff, predicted_diff))
            return False

        return True

    def get_current_rotation(self):
        trans = self.buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(10))
        quat = trans.transform.rotation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def update_joint_states(self, msg):
        self.joint_states = msg