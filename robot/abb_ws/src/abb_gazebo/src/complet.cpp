/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cstdlib>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "complet");
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

  //moveIt~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();


  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name="ompl_interface/OMPLPlanner";

 // if (!node_handle.getParam("/planning_plugin", planner_plugin_name))
 //   ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  

  sleep_time.sleep();
 
  // Pose Goal
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 1.0;
  pose.pose.position.z = 1.0;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

 
  req.group_name = "manipulator";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

 
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /////////////////////////////////////////////////
  /////////////////////////////////////////////////
  /////////////////////////////////////////////////
  /////////////////////////////////////////////////
  // to call the service of setModelConfiguration
  //
 
  ros::ServiceClient client = node_handle.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  gazebo_msgs::SetModelConfiguration srv;

  srv.request.model_name = "abb_irb2400";
  srv.request.urdf_param_name = "robot_description";

  
  std::vector<std::__cxx11::basic_string<char>> j;
  j.push_back("joint_1");
  j.push_back("joint_2");
  j.push_back("joint_3");
  j.push_back("joint_4");
  j.push_back("joint_5");
  j.push_back("joint_6");
  srv.request.joint_names =j;

  std::vector<double> p;
  robot_state::RobotState s=res.trajectory_->getLastWayPoint();
  for (int j_idx=1;j_idx<=6;j_idx++){
     p.push_back(*(s.getJointPositions("joint_"+std::to_string(j_idx))));
     
  } 
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

 

  // END_TUTORIAL
  sleep_time.sleep();
/*
  moveit::planning_interface::MoveGroupInterface group("manipulator");
// ...
robot_state::RobotStatePtr state = group.getCurrentState();
Eigen::Affine3d transform = state->getFrameTransform("tool0");
ROS_INFO("%f",transform);
*/
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
