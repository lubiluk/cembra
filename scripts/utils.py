
import actionlib
import control_msgs.msg
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import gazebo_msgs.msg
import gazebo_msgs.srv
from PIL import Image

import config

def move_to_start_pose():
    move_gripper_to_start_pose()
    move_head_to_start_pose()
    move_arm_to_start_pose()


def move_arm_to_start_pose():
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # fill ROS message
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = config.START_ARM_JOINT_NAMES
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = config.START_ARM_JOINT_ANGLES
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]
    goal.trajectory = traj

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

def move_gripper_to_start_pose():
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # fill ROS message
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ['hand_motor_joint']
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [config.START_GRIPPR_JOINT_ANGLE]
    p.velocities = [0]
    p.effort = [config.START_GRIPPR_JOINT_EFFORT]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]
    goal.trajectory = traj

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

def move_head_to_start_pose():
    move_head(config.START_HEAD_PAN, config.START_HEAD_TILT)

def move_head(pan, tilt):
    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/head_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # fill ROS message
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [pan, tilt]
    p.velocities = [0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]
    goal.trajectory = traj

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

def get_image():
    msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', sensor_msgs.msg.Image)
    img = Image.frombytes('RGB', (msg.width, msg.height), msg.data)
    img.save('camera.jpg')

    return msg

def set_model_position(model_name, x, y):
    # Set object positions
    try:
        # Set coke_can1 state
        state = gazebo_msgs.msg.ModelState()
        state.pose.position.x = x
        state.pose.position.y = y
        state.reference_frame = "world"
        state.model_name = model_name

        perform_action = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        response = perform_action(state)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))
        
