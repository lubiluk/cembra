#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{
class TfBroadcastPlugin : public WorldPlugin
{
private:
  physics::WorldPtr world;
  event::ConnectionPtr updatec_connection;
  tf2_ros::TransformBroadcaster broadcaster;
public:
  TfBroadcastPlugin() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->world = _world;
    this->updatec_connection = event::Events::ConnectWorldUpdateEnd(
          std::bind(&TfBroadcastPlugin::OnUpdate, this));

    ROS_INFO("Hello World!");
  }

  void OnUpdate()
  {
      auto hsrb = this->world->ModelByName("hsrb");

      if (!hsrb) return;

      auto hsrb_pose = hsrb->WorldPose();
      
      geometry_msgs::TransformStamped transform;
    
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = "world";
      transform.child_frame_id = "base_footprint";
      transform.transform.translation.x = hsrb_pose.Pos().X();
      transform.transform.translation.y = hsrb_pose.Pos().Y();
      transform.transform.translation.z = hsrb_pose.Pos().Z();
    
      transform.transform.rotation.x = hsrb_pose.Rot().X();
      transform.transform.rotation.y = hsrb_pose.Rot().Y();
      transform.transform.rotation.z = hsrb_pose.Rot().Z();
      transform.transform.rotation.w = hsrb_pose.Rot().W();

      this->broadcaster.sendTransform(transform);

      auto cube = this->world->ModelByName("wood_cube_5cm");
      auto cube_pose = cube->WorldPose();
      
      geometry_msgs::TransformStamped transform2;
    
      transform2.header.stamp = ros::Time::now();
      transform2.header.frame_id = "world";
      transform2.child_frame_id = "wood_cube_5cm";
      transform2.transform.translation.x = cube_pose.Pos().X();
      transform2.transform.translation.y = cube_pose.Pos().Y();
      transform2.transform.translation.z = cube_pose.Pos().Z();
    
      transform2.transform.rotation.x = cube_pose.Rot().X();
      transform2.transform.rotation.y = cube_pose.Rot().Y();
      transform2.transform.rotation.z = cube_pose.Rot().Z();
      transform2.transform.rotation.w = cube_pose.Rot().W();

      this->broadcaster.sendTransform(transform2);
  }

};
GZ_REGISTER_WORLD_PLUGIN(TfBroadcastPlugin)
}