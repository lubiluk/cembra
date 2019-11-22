#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cembra_controller");
    ROS_INFO_STREAM("Using MoveIt");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    ROS_INFO_STREAM("Using MoveIt");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO_STREAM("Using MoveIt");
    planning_scene::PlanningScene planning_scene(kinematic_model);
    ROS_INFO_STREAM("Using MoveIt");
    collision_detection::CollisionRequest collision_request;
    ROS_INFO_STREAM("Using MoveIt");
    collision_detection::CollisionResult collision_result;
    ROS_INFO_STREAM("Using MoveIt");
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    ROS_INFO_STREAM("Using MoveIt");

    ros::shutdown();

    return 0;
}