#include "joy_cmd.h"


#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "joy_cmd"
#define CHANGE_MODE 0
#define REDUCE_ANGULAR 1
#define INCREASE_ANGULAR 2
#define INCREASE_LINEAR 3
#define REDUCE_LINEAR 4
#define VEL_INC 0.2

void JoyCmd::Prepare() {
    // obtain a subscriber to the topic joy, where the input are read
    joySub = handle.subscribe("joy", 10, &JoyCmd::joyCallback, this);

    // obtain a publisher to the cmd_joy topic where turtlebot read the command
    cmdPub = handle.advertise<geometry_msgs::Twist>("cmd_joy", 10);
    
    this -> switchState = true; //default auto
    
    //wait until the mux node is running
    ros::service::waitForService("mode",-1);
    // obtain the service client
    client = handle.serviceClient<ros_turtlebot_pilot::SetMode>("mode");
    ros_turtlebot_pilot::SetMode msg;
    msg.request.mode = this->switchState;
    client.call(msg);
    
    // obtain the values max_linear and max_angular from the parameter server and 
    // store them in the maxLinear and maxAngular variable
    handle.param("/max_linear", maxLinear, 0.5);
    handle.param("/max_angular", maxAngular, 0.5);    
    
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void JoyCmd::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {

    //set the mode through the service offered by mux
    if(msg->buttons[CHANGE_MODE]){
	this->switchState = !this->switchState;
	ros_turtlebot_pilot::SetMode msg;
	msg.request.mode = this->switchState;
	client.call(msg);
    }
    
    //change the maxLinear speed
    if (msg->buttons[INCREASE_LINEAR]) {
	this->maxLinear += this->maxLinear * 0.1;
    }
    if (msg->buttons[REDUCE_LINEAR]) {
	this->maxLinear -= this->maxLinear * 0.1;
    }
    
    //change the maxAngular speed
    if (msg->buttons[INCREASE_ANGULAR]){
	this->maxAngular += this->maxAngular * 0.1;
    }
    if (msg->buttons[REDUCE_ANGULAR]){
	this->maxAngular -= this->maxAngular * 0.1;
    }

    //setting the velocity read from the message
    double targetVel = maxLinear * msg->axes[1];
    if(out.linear.x < targetVel) {
	out.linear.x = std::min(out.linear.x + VEL_INC, targetVel);
    }
    else{
	out.linear.x = std::max(out.linear.x - VEL_INC, targetVel);
    }
    double targetAngular = maxAngular * msg->axes[3];
    if(out.angular.z < targetAngular) {
	out.angular.z = std::min(out.angular.z + VEL_INC, targetAngular);
    }
    else{
	out.angular.z = std::max(out.angular.z - VEL_INC, targetAngular);
    }
}

void JoyCmd::RunContinuously() {
    ros::spin();
}

void JoyCmd::RunPeriodically () {
    ros::Rate r(10);
    ros::Rate r1(1/RUN_PERIOD_DEFAULT);
    
    while(ros::ok()) {
        ros::spinOnce();
        cmdPub.publish(out);
        r.sleep();
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  JoyCmd mNode;
   
  mNode.Prepare();
  //mNode.RunContinuously();
  mNode.RunPeriodically();
  
  return (0);
}










