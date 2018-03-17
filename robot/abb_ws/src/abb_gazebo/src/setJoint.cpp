#include "ros/ros.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_setJoint_test");
 
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  gazebo_msgs::SetModelConfiguration srv;

  srv.request.model_name = "abb_irb2400";
  srv.request.urdf_param_name = "robot_description";
  std::vector<std::__cxx11::basic_string<char>> j;
  j.push_back("joint_2");
  srv.request.joint_names =j;
  std::vector<double> p;
  p.push_back(-1.233);
  srv.request.joint_positions =p;
  if (client.call(srv))
  {
    ROS_INFO("%s",srv.response.status_message);
  }
  else
  {
    ROS_ERROR("Failed to call service set_model_configuration");
    return 1;
  }

  return 0;
}
