
import gazebo_msgs.srv
import geometry_msgs.msg
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_srvs.srv


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

def get_model_position(model):
    try:
        service = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)
        response = service(model, 'link')

        q = (
            response.pose.orientation.x,
            response.pose.orientation.y,
            response.pose.orientation.z,
            response.pose.orientation.w)
        p = response.pose.position
        e = euler_from_quaternion(q)

        return (p.x, p.y, e[2])
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))
        return None

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