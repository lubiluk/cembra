#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_INFO("%d", msg->name[0].size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 1, jointStateCallback);

    ros::spin();

    return 0;
}