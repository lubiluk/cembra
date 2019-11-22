
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from constants import *
import copy

class VelocityValidator(object):
    _requested_velocities = None
    _requested_twist = None
    current_joint_states = None
    predicted_joint_states = {}
    predicted_link_status = {}
    reference_rotation = None
    current_rotation = None
    predicted_rotation = None
    velocities_ok = None
    twist_ok = None

    listener = None
    kinematics = {}

    @property
    def requested_velocities(self):
        return self._requested_velocities

    @requested_velocities.setter
    def requested_velocities(self, velocities):
        self._requested_velocities = velocities
        self.predict_joint_states()
        self.check_requested_velocities()

    @property
    def requested_twist(self):
        return self._requested_twist

    @requested_twist.setter
    def requested_twist(self, twist):
        self._requested_twist = twist
        self.predict_rotation()
        self.check_requested_twist()

    def __init__(self):
        robot = URDF.from_parameter_server()
        for link in PALM_LINKS:
            self.kinematics[link] = KDLKinematics(robot, REFERENCE_FRAME, link)
        for link in GRIPPER_LINKS:
            self.kinematics[link] = KDLKinematics(robot, REFERENCE_FRAME, link)

    def is_link_trajectory_ok(self, link):
        is_gripper = link in GRIPPER_LINKS
        upper_bound = MAX_GRIPPER_HEIGHT if is_gripper else MAX_PALM_HEIGHT
        lower_bound = MIN_GRIPPER_HEIGHT if is_gripper else MIN_PALM_HEIGHT
        kinematics = self.kinematics[link]
            
        (current_angles, _, _) = kinematics.extract_joint_state(self.current_joint_states)
        current_pose = kinematics.forward(current_angles)
        current_height = current_pose[2,3]

        (predicted_angles, _, _) = kinematics.extract_joint_state(self.predicted_joint_states)
        predicted_pose = kinematics.forward(predicted_angles)
        predicted_height = predicted_pose[2,3]

        if predicted_height <= lower_bound and predicted_height <= current_height:
            print("arm lower bound violation: {} ({} => {})".format(link, current_height, predicted_height))
            return False

        if predicted_height >= upper_bound and predicted_height >= current_height:
            print("arm upper bound violation: {} ({} => {})".format(link, current_height, predicted_height))
            return False

        return True

    def predict_joint_states(self):
        if not self.current_joint_states:
            return
        if not self.predicted_joint_states:
            self.predicted_joint_states = copy.deepcopy(self.current_joint_states)

        # work on mutable copies
        predicted_joint_positions = list(self.current_joint_states.position)
        predicted_joint_velocities = list(self.current_joint_states.velocity)

        for i, n in enumerate(self.requested_velocities.name):
            v = self.requested_velocities.velocity[i]
            index = self.predicted_joint_states.name.index(n)
            # desired angular velocity assuming 1 second drift
            predicted_joint_positions[index] += v
            predicted_joint_velocities[index] = v

        self.predicted_joint_states.position = tuple(predicted_joint_positions)
        self.predicted_joint_states.velocity = tuple(predicted_joint_velocities)

    def check_requested_velocities(self):
        ok = True

        for link in (PALM_LINKS + GRIPPER_LINKS):
            self.predicted_link_status[link] = self.is_link_trajectory_ok(link)
            if not self.predicted_link_status[link]:
                ok = False
        
        self.velocities_ok = ok

    def predict_rotation(self):
        self.predicted_rotation = self.current_rotation + self.requested_twist.angular.z

    def check_requested_twist(self):
        base = math.pi 
        current_diff = base - abs(abs(self.reference_rotation - self.current_rotation) % (2 * base) - base)
        predicted_diff = base - abs(abs(self.reference_rotation - self.predicted_rotation) % (2 * base) - base)
        
        if predicted_diff >= MAX_ROTATION and predicted_diff >= current_diff:
            print("base violation ({} => {})".format(current_diff, predicted_diff))
            self.twist_ok = False

        self.twist_ok = True

