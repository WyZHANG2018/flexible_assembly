#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>
#include "icosahedron.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

#define PI 3.14159265359
int image_num=0;

void imageCallback_depth(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  
  //try
   // {
      std::string e=msg->encoding;
      std::cout<<e<<std::endl;
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      std::cout<<cv_ptr->image.type()<<std::endl;

      cv::Mat img = cv_ptr->image;
//float tmp=img.at<float>(55,55);
//std::cout <<img;

for(int i=0; i<img.rows; i++)
    for(int j=0; j<img.cols; j++) 
        if(!std::isnan(img.at<float>(i,j)))
        std::cout <<img.at<float>(i,j)<<std::endl;
/*  
      if (cv::imwrite("/home/weiyizhang/grasp/robot/abb_ws/depth"+std::to_string(image_num)+".png",cv_ptr->image)){
          ROS_INFO("depth image saved ");
        }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  //ROS_INFO("the encoding of the image is: [%s]!!!!!!!!!!!!!!!!!", msg->encoding.data());
*/
}


void imageCallback_rgb(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      if (cv::imwrite("/home/weiyizhang/grasp/robot/abb_ws/rgb"+std::to_string(image_num)+".jpg",cv_ptr->image)){
          ROS_INFO("rgb image saved ");
        }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  //ROS_INFO("the encoding of the image is: [%s]!!!!!!!!!!!!!!!!!", msg->encoding.data());
}



int main(int argc, char** argv)
{

//init node
ros::init(argc, argv, "nuts");
ros::AsyncSpinner spinner(1);
ros::NodeHandle node_handle;
ros::WallDuration sleep_time(2.0);//minimal time ?
//spawn model
ros::ServiceClient client_spawner = node_handle.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
gazebo_msgs::SpawnModel srv_spawner;
std::string model_xml;
if (!(node_handle.getParam("part_description",model_xml)))
ROS_INFO("failed to get the param T_T");
srv_spawner.request.model_xml =model_xml;
srv_spawner.request.model_name = "nut";
//delete model
ros::ServiceClient client_delete = node_handle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
gazebo_msgs::DeleteModel srv_delete;
srv_delete.request.model_name="nut";
//listen to image topic 
ros::Subscriber sub_image_depth = node_handle.subscribe("/camera/depth/image_depth", 1, imageCallback_depth);
ros::Subscriber sub_image_rgb = node_handle.subscribe("/camera/depth/image_rgb", 1, imageCallback_rgb);

//pose of camera in world frame
Eigen::Matrix3f m_cam;
m_cam(0,0)=-1.0;m_cam(0,1)=0.0;m_cam(0,2)=0.0;
m_cam(1,0)=0.0;m_cam(1,1)=-1.0;m_cam(1,2)=0.0;
m_cam(2,0)=0.0;m_cam(2,1)=0.0;m_cam(2,2)=1.0;
float x_cam=1.50;
float y_cam=0.0;
float z_cam=1.35;
//set initial pose
Eigen::Matrix3f m;
Eigen::Quaternionf q_final;
Eigen::Quaternionf q;
Eigen::Quaternionf q_inplan;
std::vector<geometry_msgs::Point>  vp=all_vertices(2);
std::vector<geometry_msgs::Point>::iterator it;
geometry_msgs::Point p_tmp;
geometry_msgs::Pose pose_nut;
float Xx_tmp,Xy_tmp,Xz_tmp;
float Yx_tmp,Yy_tmp,Yz_tmp;
float Zx_tmp,Zy_tmp,Zz_tmp;
float x_t,y_t,z_t;
float x_range=0.05;
float y_range=0.05;
float z_range=0.05;
int num_range=5;
float ratio;
//iter over vertices
for(it = vp.begin(); it != vp.end(); it++){
p_tmp=*it;
//axis X
Xx_tmp=p_tmp.x;
Xy_tmp=p_tmp.y;
Xz_tmp=p_tmp.z;
ratio=-1.0/sqrt(Xx_tmp*Xx_tmp+Xy_tmp*Xy_tmp+Xz_tmp*Xz_tmp);
Xx_tmp*=ratio;
Xy_tmp*=ratio;
Xz_tmp*=ratio;
//axis Y z=(0 0 1)^x=(x,y,z) 
Yx_tmp=-Xy_tmp;
Yy_tmp=Xx_tmp;
Yz_tmp=0.0;
//axis Z
Zx_tmp=Xy_tmp*Yz_tmp-Yy_tmp*Xz_tmp;
Zy_tmp=Xz_tmp*Yx_tmp-Yz_tmp-Xx_tmp;
Zz_tmp=Xx_tmp*Yy_tmp-Yx_tmp*Xy_tmp;
// rotation which represents the pose of the part frame in the camera frame
m(0,0)=Xx_tmp;m(0,1)=Xy_tmp;m(0,2)=Xy_tmp;
m(1,0)=Yx_tmp;m(1,1)=Yy_tmp;m(1,2)=Yy_tmp;
m(2,0)=Zx_tmp;m(2,1)=Zy_tmp;m(2,2)=Zy_tmp;
//pose of the part frame in the word frame
m=m_cam*m;
//convert to quaternion representation
q=m;
for (int num_t=0;num_t<=num_range;num_t++){
x_t=0.56-x_range+2.0*x_range/num_range*num_t;
y_t=0.0-y_range+2.0*y_range/num_range*num_t;
z_t=1.35-z_range+2.0*z_range/num_range*num_t;
for (int num_r=0;num_r<=23;num_r++){
/*
q_inplan.x=x_cam;
q_inplan.y=y_cam;
q_inplan.z=z_cam;
q_inplan.w=2.0*PI*num_r/24.0;
*/
float semiangle=PI*(float)num_r/24.0;
q_inplan=Eigen::Quaternionf(std::cos(semiangle),std::sin(semiangle),0.0,0.0);
q_final=q_inplan*q;

//angle/2 in order to project 0~2PI into -1~1


pose_nut.orientation.x=q_final.x();
pose_nut.orientation.y=q_final.y();
pose_nut.orientation.z=q_final.z();
pose_nut.orientation.w=q_final.w();
pose_nut.position.x =x_t;
pose_nut.position.y =y_t;
pose_nut.position.z =z_t;

image_num++;
//spawn model
srv_spawner.request.initial_pose=pose_nut;

if (client_spawner.call(srv_spawner))
{
ROS_INFO("Successfully to call service spawn_urdf_model");
}
else
{
ROS_ERROR("Failed to call service spawn_urdf_model");
return 1;
}
spinner.start();
sleep_time.sleep();
spinner.stop();
//take photo
//delete model
if (client_delete.call(srv_delete))
{
ROS_INFO("Successfully to call service delete_model");
}
else
{
ROS_ERROR("Failed to call service delete_model");
return 1;
}
sleep_time.sleep();
}


}


// 
}











ROS_INFO("Done");

return 0;
}
