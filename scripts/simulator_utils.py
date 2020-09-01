
import gazebo_msgs.srv
import gazebo_msgs.msg
import geometry_msgs.msg
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_srvs.srv
import math as m

def set_model_position(model_name, x, y, yaw):
    q = quaternion_from_euler(0, 0, yaw)

    # Set object positions
    try:
        state = gazebo_msgs.msg.ModelState()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.orientation = geometry_msgs.msg.Quaternion(q[0], q[1], q[2], q[3])
        state.reference_frame = "world"
        state.model_name = model_name

        perform_action = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        response = perform_action(state)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

def get_model_state(model):
    try:
        service = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)
        response = service(model, 'world')

        return response
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))
        return None

def get_model_position(model):
    response = get_model_state(model)

    q = (
        response.pose.orientation.x,
        response.pose.orientation.y,
        response.pose.orientation.z,
        response.pose.orientation.w)
    p = response.pose.position
    e = euler_from_quaternion(q)

    return (p.x, p.y, e[2])

def get_link_state(link):
    try:
        service = rospy.ServiceProxy('/gazebo/get_link_state', gazebo_msgs.srv.GetLinkState)
        response = service(link, 'world')

        return response.link_state
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))
        return None

def get_link_distance(link1, link2):
    state1 = get_link_state(link1)
    state2 = get_link_state(link2)

    return m.sqrt((state2.pose.position.x - state1.pose.position.x)**2 + 
        (state2.pose.position.y - state1.pose.position.y)**2 +
        (state2.pose.position.z - state1.pose.position.z)**2)

def pause():
    try:
        service = rospy.ServiceProxy('/gazebo/pause_physics', std_srvs.srv.Empty)
        service()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

def resume():
    try:
        service = rospy.ServiceProxy('/gazebo/unpause_physics', std_srvs.srv.Empty)
        service()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

def get_physics_properties():
    try:
        service = rospy.ServiceProxy('/gazebo/get_physics_properties', gazebo_msgs.srv.GetPhysicsProperties)
        return  service()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

def go_turbo():
    try:
        props = get_physics_properties()
        props.max_update_rate = 0

        service = rospy.ServiceProxy('/gazebo/set_physics_properties', gazebo_msgs.srv.SetPhysicsProperties)
        service(props.time_step, props.max_update_rate, props.gravity, props.ode_config)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))