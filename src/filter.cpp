#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tmc_msgs/JointVelocity.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

planning_scene::PlanningScenePtr scene;
ros::Publisher pub;
ros::ServiceClient planning_scene_diff_client;

bool isVelocitySafe(const tmc_msgs::JointVelocity &msg);

void velocityCallback(const tmc_msgs::JointVelocity &msg)
{
    if (isVelocitySafe(msg))
    {
        pub.publish(msg);
        ROS_INFO("Safe velocity");
        return;
    }

    ROS_INFO("Unsafe velocity");

    return;
}

bool isVelocitySafe(const tmc_msgs::JointVelocity &msg)
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_state::RobotState &current_state = scene->getCurrentStateNonConst();

    for (int i = 0; i < msg.name.size(); ++i)
    {
        auto name = msg.name[i];
        auto positions = current_state.getJointPositions(name);
        auto vel = msg.velocity[i];
        auto pos = positions[0];

        pos += vel; // Assume 1 sec drift

        current_state.setJointPositions(name, &pos);
    }

    collision_result.clear();
    collision_request.distance = true;
    scene->checkCollision(collision_request, collision_result);

    ROS_INFO("Collision distance %f", collision_result.distance);

    auto isSafe = collision_result.distance >= 0.02;

    if (!isSafe)
    {
        ROS_INFO_STREAM("Not safe");
    }

    if (collision_result.collision) {
        ROS_INFO("In collision %f", collision_result.distance);
    }

    return isSafe;
}

void insertGroundPlane()
{
    moveit_msgs::CollisionObject object;
    object.header.frame_id = "base_footprint";

    shape_msgs::SolidPrimitive shape;
    shape.type = shape_msgs::SolidPrimitive::BOX;
    shape.dimensions = {10.0 , 10.0, 1.0};

    geometry_msgs::Pose pose;
    pose.position.z = -0.5;
    pose.orientation.w = 1.0;

    object.primitives.push_back(shape);
    object.primitive_poses.push_back(pose);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter");

    // Setup MoveIt
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model));

    // Setup
    ros::NodeHandle n;

    ROS_INFO("Creating ground plane.");
    planning_scene_diff_client = n.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();

    insertGroundPlane();

    pub = n.advertise<tmc_msgs::JointVelocity>("/hsrb/pseudo_velocity_controller/ref_joint_velocity", 1);

    while (pub.getNumSubscribers() == 0)
    {
        ros::Duration(0.1).sleep();
    }

    ros::Subscriber sub = n.subscribe("/cembra/velocity", 1, velocityCallback);
    ROS_INFO("Ready to filter velocity.");
    ros::waitForShutdown();

    return 0;
}