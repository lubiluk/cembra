#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

const double MAX_ROTATION = M_PI / 2;

ros::Publisher pub;
tf2_ros::Buffer *tfBuffer;
tf2_ros::TransformListener *tfListener;

double reference_rotation;
double current_rotation;

const double base = M_PI;

double getCurrentRotation()
{
    geometry_msgs::TransformStamped trans;

    try{
      trans = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    
    return tf::getYaw(trans.transform.rotation);
}

bool isTwistSafe(const geometry_msgs::Twist &msg)
{
    double velocity = msg.angular.z;
    double current_diff = base - fabs(fmod(fabs(reference_rotation - current_rotation), (2.0 * base)) - base);

    // Sample future positions
    for (int j = 1; j <= 10; ++j)
    {
        double predicted_diff = current_rotation + velocity * 0.1 * j;

        bool is_safe = (predicted_diff >= MAX_ROTATION && predicted_diff >= current_diff);

        if (!is_safe)
        {
            ROS_INFO_STREAM("Twist not safe");
            return false;
        }
    }

    return true;
}

void twistCallback(const geometry_msgs::Twist &msg)
{
    current_rotation = getCurrentRotation();
    
    if (isTwistSafe(msg))
    {
        pub.publish(msg);
        // ROS_INFO("Safe twist");
        return;
    }

    // ROS_INFO("Unsafe velocity");

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_filter");

    tfBuffer = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuffer);

    // Setup reference rotation
    reference_rotation = getCurrentRotation();

    // Setup
    ros::NodeHandle n;

    pub = n.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 1);

    while (pub.getNumSubscribers() == 0)
    {
        ros::Duration(0.1).sleep();
    }

    ros::Subscriber sub = n.subscribe("/cembra/twist", 1, twistCallback);
    ROS_INFO("Ready to filter twist.");
    ros::spin();

    return 0;
}