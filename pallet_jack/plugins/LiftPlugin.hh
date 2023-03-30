#ifndef _GAZEBO_LIFT_PLUGIN_HH
#define _GAZEBO_LIFT_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <stdio.h>
#include <boost/bind.hpp>

namespace gazebo {
  class LiftPlugin : public ModelPlugin{
    
  public: LiftPlugin();
  public: virtual ~LiftPlugin();
  public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
   // update world callback
  public: void OnUpdateWorld(const common::UpdateInfo & /*_info*/);
  private: void OnUpdate(const std_msgs::Bool::ConstPtr& msg); 
 
  
  private: ros::Subscriber subscriber;
  // lift 
  private: physics::LinkPtr lift;
  private: physics::JointPtr prismaticJoint;
  private: common::PID jointPid;
  // last msg received from topic
  private: bool lastMsg;
  private: physics::ModelPtr model;
  private: common::Time prevUpdateTime;
  private: event::ConnectionPtr updateConnection;
  private: ros::Time load_timestamp;
  
};
}

#endif