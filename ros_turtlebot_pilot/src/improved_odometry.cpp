#include "improved_odometry.h"

#define NAME_OF_THIS_NODE "improved_odometry"

#define WHEEL_DISTANCE 0.23
#define WHEEL_RADIUS 0.04

#define RATE 1000

#define PI 3.141592
// equal to 0,002 degrees per second
#define EXACT_ANGULAR_THRESHOLD 0.00005 

void ImprovedOdometry::mapInit()
{
    modeMap["euler"] = Mode::EULER;
    modeMap["runge-kutta"] = Mode::RUNGE_KUTTA;
    modeMap["exact"] = Mode::EXACT;
    
    methodMap[Mode::EULER] = &ImprovedOdometry::odomEuler;
    methodMap[Mode::RUNGE_KUTTA] = &ImprovedOdometry::odomRungeKutta;
    methodMap[Mode::EXACT] = &ImprovedOdometry::odomExact;
}

void ImprovedOdometry::Prepare() {
    mapInit();
    currentPosition.theta=0;
    currentPosition.x=0;
    currentPosition.y=0;
    currentIteration = 0;
	
    ros::NodeHandle localHandler("~");
    
    // get the data_num param and greate the data arrays
    localHandler.param<int>("data_num", this->dataNumber, /*default*/1);
    
    //get the mode parameter and set this->mode
    std::string modeParam;
    localHandler.param<std::string>("odom_method", modeParam, /*default*/ "euler");
    try {
        this->mode = modeMap.at(modeParam);
    }catch (const std::out_of_range& oor){
        this->mode = Mode::EULER;
    }
    
    //subscribe to odom callback
    jointStateSub = handler.subscribe<nav_msgs::Odometry>("odom", 10, &ImprovedOdometry::odometryCallback, this);
    
    //advertise improved_odom topic
    odomPub = handler.advertise<nav_msgs::Odometry>("improved_odom", 10);
	
	std::cout<<" Prepare method finished correctly\n\n";
}

void ImprovedOdometry::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	linearVelocities.push_back(msg->twist.twist.linear.x);
	angularVelocities.push_back(msg->twist.twist.angular.z);
	currentIteration++;
}

void ImprovedOdometry::odomEuler(float linearVel, float angularVel)
{
    //compute delta T
    double delta = (ros::Time::now() - this->lastTS).toSec();
    //compute current position
    this->currentPosition.x += linearVel*delta*std::cos(currentPosition.theta);
    this->currentPosition.y += linearVel*delta*std::sin(currentPosition.theta);
    this->currentPosition.theta += angularVel*delta;
}

void ImprovedOdometry::odomExact(float linearVel, float angularVel)
{
    //compute current position, if lower than threshold call runge kutta
	if(std::abs(angularVel)>EXACT_ANGULAR_THRESHOLD){
		//compute delta T
		double delta = (ros::Time::now() - this->lastTS).toSec();
		double lastOrientation = this->currentPosition.theta;
		this->currentPosition.theta+= angularVel*delta;
		this->currentPosition.x += (linearVel/angularVel)*(std::sin(currentPosition.theta)-std::sin(lastOrientation));
		this->currentPosition.y -= (linearVel/angularVel)*(std::cos(currentPosition.theta)-std::cos(lastOrientation));
	}
	else{
		odomRungeKutta(linearVel,angularVel);
	}
}

// the average orientation is used
void ImprovedOdometry::odomRungeKutta(float linearVel, float angularVel)
{
	 //compute delta T
    double delta = (ros::Time::now() - this->lastTS).toSec();
    //compute current position
    this->currentPosition.x += linearVel*delta*std::cos(currentPosition.theta + angularVel*delta/2);
    this->currentPosition.y += linearVel*delta*std::sin(currentPosition.theta + angularVel*delta/2);
    this->currentPosition.theta += angularVel*delta;
}

void ImprovedOdometry::Loop()
{
    //first time settign the lastTS
    this->lastTS = ros::Time::now();
    
     ros::Rate r(RATE);
    
    while(ros::ok()) {
	
	ros::spinOnce();
	
	if(currentIteration >= dataNumber) {
		// first get the left and right angular vel
	    float linear = std::accumulate(linearVelocities.begin(), linearVelocities.end(), 0.0)/linearVelocities.size();
	    float angular = std::accumulate(angularVelocities.begin(), angularVelocities.end(), 0.0)/angularVelocities.size();
	    
	    // update this->currentPosition
	    (this->*methodMap.at(this->mode))(linear, angular); //methodMap.at(this->mode) returns the pointer to the correct method to call
	    
	    //update lastTS, will be used in the next iteration in the odometry method
	    this->lastTS = ros::Time::now();
	    
	    //publish the computed odometry values
	    nav_msgs::Odometry msg;
	    msg.header.stamp = ros::Time::now();
	    msg.header.frame_id = "improved_odom";
	    msg.pose.pose.position.y = this->currentPosition.y;
	    msg.pose.pose.position.x = this->currentPosition.x;
	    msg.pose.pose.position.z = 0;
		// what gets createQuaternionMsgFromYaw ? Radians or eulerian angle?
	    msg.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(currentPosition.theta);
		// notice that these are velocities in the robot frame, so vy is zero
	    msg.twist.twist.angular.x = 0;
	    msg.twist.twist.angular.y = 0;
	    msg.twist.twist.angular.z = angular;
	    msg.twist.twist.linear.x = linear;
	    msg.twist.twist.linear.y = 0;
	    msg.twist.twist.linear.z = 0;
	    
	    odomPub.publish<nav_msgs::Odometry>(msg);
	    
	    //clear the vector to accumulate a new set of velocities
	    linearVelocities.clear();
	    angularVelocities.clear();
	    
	    currentIteration=0;
	}
	
	r.sleep();
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    ImprovedOdometry node;
    node.Prepare();
    node.Loop();
}
