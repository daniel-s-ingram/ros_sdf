#ifndef _ROS_SDF_HH_
#define _ROS_SDF_HH_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

namespace gazebo
{
  class ROS_SDF : public ModelPlugin
  {
    public: ROS_SDF() {}
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    private: physics::ModelPtr model;
    private: physics::JointPtr *joints;
    private: common::PID pid;
    private: int num_joints;

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      if ((this->num_joints = _model->GetJointCount()) == 0)
      {
        std::cerr << "Invalid joint count, ROS_SDF plugin not loaded\n";
        return;
      }
      else
      {
        std::cout << "\nThe ROS_SDF plugin is attached to model[" << _model->GetName() << "]\n";
      }

      this->model = _model;
      this->joints = new physics::JointPtr[this->num_joints]; //need to delete this 

      for(int i = 0; i < this->num_joints; i++)
      {
        this->joints[i] = _model->GetJoints()[i];
        this->model->GetJointController()->SetPositionPID(this->joints[i]->GetScopedName(), this->pid);
      }

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      ros::SubscribeOptions so =ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/" + this->model->GetName() + "/position_cmd", 
                                                                                          100, 
                                                                                          boost::bind(&ROS_SDF::SetPosition, this, _1),
                                                                                          ros::VoidPtr(), &this->rosQueue);
      
      this->rosSub = this->rosNode->subscribe(so);
      this->rosQueueThread = std::thread(std::bind(&ROS_SDF::QueueThread, this));
    }

    public: void SetPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
      for(int i = 0; i < this->num_joints; i++)
      {
        this->model->GetJointController()->SetPositionTarget(this->joints[i]->GetScopedName(), msg->data[i]);
      }
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(ROS_SDF)
}
#endif