#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


void Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  cv::imwrite("camera.jpg",cv_ptr->image);
  ROS_INFO("h:%d w:%d",msg->height,msg->width);
  //ros::WallDuration sleep_time(60.0);
  //sleep_time.sleep();
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "imageReceiver");


  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("abb/camera1/image_raw", 1, Callback);

  ros::spin();

  return 0;
}
