#!/usr/bin/env python

import rospy
import trajectory_msgs.msg
import gazebo_msgs.msg
import cembra.srv
import simulator_utils
import robot_utils
import sensor_msgs.msg
import control_msgs.msg
import nav_msgs.msg
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import tf

TIME_STEP = 0.1
MODEL_STATE_TOPIC = '/gazebo/model_states'
BASE_STATE_TOPIC = '/hsrb/omni_base_controller/state'
JOINT_STATE_TOPIC = '/hsrb/joint_states'
ODOM_TOPIC = '/hsrb/odom'
CAMERA_REF_FRAME = 'head_rgbd_sensor_rgb_frame'

class Environment4:
    """
        An environment with a robot and one cube on a workspace.
        The robot is rewarded when it touches the cube.
        Actions are real-valued.
        Information about objects is provided directly from the simulator.
        Object pose is given relatively to robot's camera frame.
    """
    def __init__(self):
        rospy.init_node("environment_4")
        rospy.loginfo("Environment 4")

        self._collision_registered = False
        self._joint_states = {}
        self._object_pose = None
        self._joint_state_msg = None
        self._base_state_msg = None
        self._odom_msg = None

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
        self._model_state_sub = rospy.Subscriber(
            MODEL_STATE_TOPIC, 
            gazebo_msgs.msg.ModelStates, self._handle_model_state)
        self._joint_state_sub = rospy.Subscriber(
            JOINT_STATE_TOPIC, 
            sensor_msgs.msg.JointState, self._handle_joint_state)
        self._base_state_sub = rospy.Subscriber(
            BASE_STATE_TOPIC, 
            control_msgs.msg.JointTrajectoryControllerState, self._handle_base_state)
        self._odom_sub = rospy.Subscriber(
            ODOM_TOPIC,
            nav_msgs.msg.Odometry, self._handle_odometry
        )

        rospy.loginfo("Waiting for controllers to attach")
        # wait to establish connection between the controller
        while (self._arm_pub.get_num_connections() == 0 
                or self._base_pub.get_num_connections() == 0):
            rospy.sleep(0.1)

        self.reset_srv = rospy.Service(
            '/cembra/reset', cembra.srv.Reset4, self._handle_reset)
        self.action_srv = rospy.Service(
            '/cembra/action', cembra.srv.Action4, self._handle_action)
        
        simulator_utils.resume()
        simulator_utils.go_turbo()

        # TF listener for retrieving object pose relatively to camera frame
        # self._tf_buffer = tf2_ros.Buffer()
        # self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # self._tf_buffer.lookup_transform("base_footprint", CAMERA_REF_FRAME, rospy.Time.now(), rospy.Duration(4.0))

        self._handle_model_state(
            rospy.wait_for_message(MODEL_STATE_TOPIC, gazebo_msgs.msg.ModelStates))
        self._handle_joint_state(
            rospy.wait_for_message(JOINT_STATE_TOPIC, sensor_msgs.msg.JointState))
        self._handle_base_state(
            rospy.wait_for_message(BASE_STATE_TOPIC, control_msgs.msg.JointTrajectoryControllerState))
        self._handle_odometry(
            rospy.wait_for_message(ODOM_TOPIC, nav_msgs.msg.Odometry))

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

        return (self._object_pose, self._joint_state_msg, self._base_state_msg, self._odom_msg)

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

        return (self._object_pose, self._joint_state_msg, self._base_state_msg, self._odom_msg, reward, is_done)

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

    def _handle_joint_state(self, msg):
        self._joint_state_msg = msg

        for i in range(len(msg.name)):
            self._joint_states[msg.name[i]] = msg.position[i]

    def _handle_base_state(self, msg):
        self._base_state_msg = msg

        for i in range(len(msg.joint_names)):
            self._joint_states[msg.joint_names[i]] = msg.actual.positions[i]

    def _handle_odometry(self, msg):
        self._odom_msg = msg

    def _handle_model_state(self, msg):
        cube_pose = msg.pose[msg.name.index('wood_cube_5cm')]
        hsrb_pose = msg.pose[msg.name.index('hsrb')]
        # we need model pose in camera coordinate frame
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose.position.x = cube_pose.position.x - hsrb_pose.position.x
        pose_stamped.pose.position.y = cube_pose.position.y - hsrb_pose.position.y
        pose_stamped.pose.position.z = cube_pose.position.z - hsrb_pose.position.z

        q1_inv = [0.0, 0.0, 0.0, 0.0]
        q1_inv[0] = hsrb_pose.orientation.x
        q1_inv[1] = hsrb_pose.orientation.y
        q1_inv[2] = hsrb_pose.orientation.z
        q1_inv[3] = -hsrb_pose.orientation.w # Negate for inverse

        q2 = [0.0, 0.0, 0.0, 0.0]
        q2[0] = cube_pose.orientation.x
        q2[1] = cube_pose.orientation.y
        q2[2] = cube_pose.orientation.z
        q2[3] = cube_pose.orientation.w

        qr = tf.transformations.quaternion_multiply(q2, q1_inv)

        pose_stamped.pose.orientation.x = qr[0]
        pose_stamped.pose.orientation.y = qr[1]
        pose_stamped.pose.orientation.z = qr[2]
        pose_stamped.pose.orientation.w = qr[3]

        # pose_stamped.header.frame_id = 'base_footprint'
        # pose_stamped.header.stamp = rospy.Time(0)
        # cam_pose = self._tf_buffer.transform(pose_stamped, CAMERA_REF_FRAME)

        self._object_pose = pose_stamped.pose

def run():
    env = Environment4()
    env.start()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass