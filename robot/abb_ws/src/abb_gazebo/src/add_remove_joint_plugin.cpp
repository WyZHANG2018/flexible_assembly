#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>

namespace gazebo
{
  class GraspTest : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      common::Time::Sleep(common::Time::MSleep(10000));
      std::cout<<"load model plugin !!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
      // Store the pointer to the model
      this->model = _parent;
      this->joint =_parent->GetJoint("nut_joint");
     
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      //this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //    std::bind(&ModelPush::OnUpdate, this));
// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client",
	      ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
        this->rosNode->setCallbackQueue(&(this->rosQueue));
	// Create a named topic, and subscribe to it.
/*
	ros::SubscribeOptions so =
	  ros::SubscribeOptions::create<geometry_msgs::Pose>(
	      "part_pose",
	      1,
	      boost::bind(&GraspTest::OnRosMsg, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so); */
        this->rosSub = this->rosNode->subscribe("part_pose", 1000, &GraspTest::OnRosMsg,this);
        std::cout<<"num of publishers"<<this->rosSub.getNumPublishers()<<std::endl; 

        std::cout<<"subsciption finished!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	// Spin up the queue helper thread.
	this->rosQueueThread =
	  std::thread(std::bind(&GraspTest::QueueThread, this));
    }

	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::PoseConstPtr &_msg)
	{
           std::cout<<"receive pose message !!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
           this->joint->Attach(this->model->GetLink("base_link"),this->model->GetLink("nut_body"));
           //set parts positions
           float x=_msg->position.x;
           float y=_msg->position.y;
           float z=_msg->position.z;
           float x_o=_msg->orientation.x;
           float y_o=_msg->orientation.y;
           float z_o=_msg->orientation.z;
           float w_o=_msg->orientation.w;
           math::Pose pose(math::Vector3(x,y,z),math::Quaternion(w_o,x_o,y_o,z_o));
           this->model->SetLinkWorldPose(pose, "nut_body");
           //add force
           this->model->GetLink("gripper_finger1_finger_tip_link")->AddForce (math::Vector3(0,-10,0)); //world frame perhaps
           this->model->GetLink("gripper_finger2_finger_tip_link")->AddForce (math::Vector3(0,10,0));
           common::Time::Sleep(common::Time::MSleep(2000));
           //loose attach
           this->joint->Detach();
	}

	/// \brief ROS helper function that processes messages
    private: void QueueThread()
	{
          
	  static const double timeout = 3.0;
	  while (this->rosNode->ok())
	  {

	    this->rosQueue.callAvailable(/*ros::WallDuration(timeout)*/);
	  }
	}

    // Called by the world update start event
    //public: void OnUpdate()
    //{
      // Apply a small linear velocity to the model.
     // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    //}

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    private: gazebo::physics::JointPtr joint;

   /// \brief A node use for ROS transport
   private: std::unique_ptr<ros::NodeHandle> rosNode;

   /// \brief A ROS subscriber
   private: ros::Subscriber rosSub;

   /// \brief A ROS callbackqueue that helps process messages
   private: ros::CallbackQueue rosQueue;

   /// \brief A thread the keeps running the rosQueue
   private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GraspTest);
}
