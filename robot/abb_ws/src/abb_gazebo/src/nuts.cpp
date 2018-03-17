#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "nuts");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;
  ros::WallDuration sleep_time(15.0);
  //parts~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ros::ServiceClient client_spawner = node_handle.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel srv_spawner;

  std::string model_xml;
  if (!(node_handle.getParam("part_description",model_xml)))
   ROS_INFO("failed to get the param T_T");
  srv_spawner.request.model_xml =model_xml;

  for (int num_nuts=0;num_nuts<10;num_nuts++){
  srv_spawner.request.model_name = "nut"+std::to_string(num_nuts);
  geometry_msgs::Pose pose_nut;
  float random = ((float) rand())/ (float) RAND_MAX*0.455;
  pose_nut.position.x = random+0.76;
  random = ((float) rand())/ (float) RAND_MAX;
  pose_nut.position.y =random-0.455;
  pose_nut.position.z =1.0;
/*
  random = ((float) rand())/ (float) RAND_MAX;
  pose_nut.orientation.x = random;
  random = ((float) rand())/ (float) RAND_MAX;
  pose_nut.orientation.y = random;
  random = ((float) rand())/ (float) RAND_MAX;
  pose_nut.orientation.z = random;
  random = ((float) rand())/ (float) RAND_MAX;
  pose_nut.orientation.w = random;
*/
  srv_spawner.request.initial_pose=pose_nut;


  if (client_spawner.call(srv_spawner))
  {
    ROS_INFO("Successfully to call service spawn_urdf_model");
  }
  else
  {
    ROS_ERROR("Failed to call service spawn_urdf_model %d",num_nuts);
    return 1;
  }

  sleep_time.sleep();
  }

  ROS_INFO("Done");

  return 0;
}
