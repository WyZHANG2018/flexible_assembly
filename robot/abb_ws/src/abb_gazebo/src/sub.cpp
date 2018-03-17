#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
void chatterCallback(const geometry_msgs::PoseConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->position.x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("part_pose", 1000, chatterCallback);
  ros::spin();

  return 0;
}
