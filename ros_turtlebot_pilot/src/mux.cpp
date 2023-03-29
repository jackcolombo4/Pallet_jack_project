
#include "mux.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "mux"

void Mux::Prepare() {
    joySub = Handle.subscribe("cmd_joy", 10, &Mux::joyCallback, this);    
    autoSub = Handle.subscribe("cmd_auto", 10, &Mux::autoCallback, this);
    modeSr = Handle.advertiseService("mode", &Mux::modeService, this);
    getModeSr = Handle.advertiseService("getMode",&Mux::getMode,this);
    cmdPub = Handle.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
    
    mode = 0;
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void Mux::RunContinuously() {
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();
}

void Mux::RunPeriodically() {
    ROS_INFO("Node %s running periodically.", ros::this_node::getName().c_str());
    
    ros::Rate r(10); //10 hz
    while (ros::ok()) {
        cmdPub.publish(command);
        ros::spinOnce();
        r.sleep();
    }
}

/*
 * this callback swicht the mode 
 */
bool Mux::modeService(ros_turtlebot_pilot::SetMode::Request &req, ros_turtlebot_pilot::SetMode::Response &res) {
    if(req.mode == 0 || req.mode ==1){
      mode = req.mode;
      res.ok = true;
      // write the string corresponding to the modality
      std::cout << "service called, mode = " << mode << "\n";
      return true;
    }
    else{
      std::cout << "Mode not supported, only mode 1 and mode 0 are supported ";
      res.ok = false;
      return false;
    }
}

/**
 * returns the mode
 */
bool Mux::getMode(ros_turtlebot_pilot::GetMode::Request& req, ros_turtlebot_pilot::GetMode::Response& res)
{
  res.mode = mode;
  return true;
}


/*
 * this callback publish the command to the turtlebot only when the mode 
 * is pilot. The callback is registered on the cmd_auto topic.
*/
void Mux::joyCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mode == 0) {
        geometry_msgs::Twist out;
        out.linear.x = msg->linear.x;
        out.angular.z = msg->angular.z;
        cmdPub.publish(out);
    }
}

/*
 * this callback publish the command to the turtlebot only when the mode 
 * is automatic. The callback is registered on the cmd_joy topic.
*/
void Mux::autoCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mode == 1) {
        geometry_msgs::Twist out;
        out.linear.x = msg->linear.x;
        out.angular.z = msg->angular.z;
        cmdPub.publish(out);
    }
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  Mux mNode;
   
  mNode.Prepare();
  mNode.RunContinuously();
  //mNode.RunPeriodically();
  
  return (0);
}


