#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tmc_msgs/JointVelocity.h>

planning_scene::PlanningScene *scene;
ros::Publisher pub;

bool willCollide(const tmc_msgs::JointVelocity &msg);

void velocityCallback(const tmc_msgs::JointVelocity &msg)
{
    if (!willCollide(msg))
    {
        pub.publish(msg);
        return;
    }

    return;
}

bool willCollide(const tmc_msgs::JointVelocity &msg)
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_state::RobotState &current_state = scene->getCurrentStateNonConst();

    for(int i = 0; i < msg.name.size(); ++i) {
        auto name = msg.name[i];
        auto positions = current_state.getJointPositions(name);
        auto vel = msg.velocity[i];
        auto pos = positions[0];

        pos += vel; // Assume 1 sec drift

        current_state.setJointPositions(name, &pos);
    }

    collision_result.clear();
    scene->checkCollision(collision_request, collision_result);

    if (collision_result.collision)
    {
        ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    }

    return collision_result.collision;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cembra_controller");

    // Setup MoveIt
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    scene = new planning_scene::PlanningScene(kinematic_model);

    // Setup
    ros::NodeHandle n;

    pub = n.advertise<tmc_msgs::JointVelocity>("/hsrb/pseudo_velocity_controller/ref_joint_velocity", 1);

    ros::Subscriber sub = n.subscribe("/cembra/velocity", 1, velocityCallback);
    ROS_INFO("Ready to filter velocity.");
    ros::waitForShutdown();

    delete scene;

    return 0;
}