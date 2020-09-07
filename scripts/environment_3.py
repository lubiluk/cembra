#!/usr/bin/env python

import rospy
import trajectory_msgs.msg
import gazebo_msgs.msg
import cembra.srv
import simulator_utils
import robot_utils
import sensor_msgs.msg
import control_msgs.msg

TIME_STEP = 0.1

class Environment3:
    """
        An environment with a robot and one cube on a workspace.
        The robot is rewarded when it touches the cube.
        Actions are real-valued.
    """
    def __init__(self):
        rospy.init_node("environment_3")
        rospy.loginfo("Environment 3")

        self._collision_registered = False
        self._image = None
        self._joint_states = {}

    def start(self):
        self._arm_pub = rospy.Publisher(
            '/hsrb/arm_trajectory_controller/command', 
            trajectory_msgs.msg.JointTrajectory, queue_size=1)
        self._base_pub = rospy.Publisher(
            '/hsrb/omni_base_controller/command', 
            trajectory_msgs.msg.JointTrajectory, queue_size=1)
        self._collision_sub = rospy.Subscriber(
            '/cube_contact_sensor_state', 
            gazebo_msgs.msg.ContactsState, self._handle_collision)
        self._image_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/rgb/image_rect_color', 
            sensor_msgs.msg.Image, self._handle_image)
        self._joint_state_sub = rospy.Subscriber(
            '/hsrb/joint_states', 
            sensor_msgs.msg.JointState, self._handle_joint_state)
        self._base_state_sub = rospy.Subscriber(
            '/hsrb/omni_base_controller/state', 
            control_msgs.msg.JointTrajectoryControllerState, self._handle_base_state)

        rospy.loginfo("Waiting for controllers to attach")
        # wait to establish connection between the controller
        while (self._arm_pub.get_num_connections() == 0 
                or self._base_pub.get_num_connections() == 0):
            rospy.sleep(0.1)

        self.reset_srv = rospy.Service(
            '/cembra/reset', cembra.srv.Reset3, self._handle_reset)
        self.action_srv = rospy.Service(
            '/cembra/action', cembra.srv.Action3, self._handle_action)
        
        simulator_utils.pause()
        simulator_utils.go_turbo()

        rospy.loginfo("Environment ready")

        rospy.spin()

    def _handle_reset(self, msg):
        simulator_utils.resume()
        robot_utils.move_to_start_pose()
        simulator_utils.set_model_position('hsrb', 0, 0, 0)
        simulator_utils.set_model_position('wood_cube_5cm', 0.7, 0, 0)
        rospy.sleep(0.1)
        simulator_utils.pause()

        self._collision_registered = False

        return self._image

    def _handle_action(self, msg):
        """
        Handles action in form of 8 integers.

        - 5 for arm
        - 3 for base
        """
        velocities = msg.action
        
        simulator_utils.resume()
        self._send_velocities(velocities)
        rospy.sleep(TIME_STEP)
        d = simulator_utils.get_link_distance('hsrb::wrist_ft_sensor_frame', 'wood_cube_5cm::link')
        simulator_utils.pause()

        reward = 100 if self._collision_registered else -d
        is_done = self._collision_registered

        if is_done:
            self._send_velocities([0, 0, 0, 0, 0, 0, 0, 0])

        return (self._image, reward, is_done)

    def _send_velocities(self, velocities):
        assert(len(velocities) == 8)

        # Arm
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = [
                'arm_flex_joint',
                'arm_lift_joint',
                'arm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint'
                ]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [
            self._joint_states['arm_flex_joint'] + velocities[0] * TIME_STEP,
            self._joint_states['arm_lift_joint'] + velocities[1] * TIME_STEP,
            self._joint_states['arm_roll_joint'] + velocities[2] * TIME_STEP,
            self._joint_states['wrist_flex_joint'] + velocities[3] * TIME_STEP,
            self._joint_states['wrist_roll_joint'] + velocities[4] * TIME_STEP
            ]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Time(TIME_STEP)
        traj.points = [p]

        self._arm_pub.publish(traj)

        # Base velocity
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ['odom_x', 'odom_y', 'odom_t']
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [
            self._joint_states['odom_x'] + velocities[5] * TIME_STEP,
            self._joint_states['odom_y'] + velocities[6] * TIME_STEP,
            self._joint_states['odom_t'] + velocities[7] * TIME_STEP,
        ]
        p.velocities = [0, 0, 0]
        p.time_from_start = rospy.Time(TIME_STEP)
        traj.points = [p]

        self._base_pub.publish(traj)

    def _handle_collision(self, msg):
        for s in msg.states:
            if s.collision2_name.startswith('hsrb::hand') or s.collision2_name.startswith('hsrb::wrist'):
                self._collision_registered = True

    def _handle_image(self, msg):
        self._image = msg

    def _handle_joint_state(self, msg):
        for i in range(len(msg.name)):
            self._joint_states[msg.name[i]] = msg.position[i]

    def _handle_base_state(self, msg):
        for i in range(len(msg.joint_names)):
            self._joint_states[msg.joint_names[i]] = msg.actual.positions[i]

def run():
    env = Environment3()
    env.start()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass