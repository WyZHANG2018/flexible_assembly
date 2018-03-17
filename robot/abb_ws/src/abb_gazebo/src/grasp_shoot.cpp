#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("part_pose", 1000);

  ros::Rate loop_rate(10);
  ros::WallDuration sleep_time(15.0);
  int i=0;
  while (ros::ok())
  { if(i==20)break;
    i++;
    geometry_msgs::Pose msg;
    msg.position.x=1.5;
    msg.position.y=0.0;
    msg.position.z=1.35;
    msg.orientation.x=0.0;
    msg.orientation.y=0.0;
    msg.orientation.z=0.0;
    msg.orientation.w=0.0;

    chatter_pub.publish(msg);
    ROS_INFO("there is %d subscriber(s)!!!!!!!!!!!",chatter_pub.getNumSubscribers());
    ros::spinOnce();
    ROS_INFO("message sent :)");
    sleep_time.sleep();
    loop_rate.sleep();
  }


  return 0;
}
