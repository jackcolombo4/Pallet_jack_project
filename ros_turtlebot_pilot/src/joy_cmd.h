#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "ros_turtlebot_pilot/SetMode.h"
#include <stdio.h>

class JoyCmd {
private:
    double maxLinear, maxAngular;
    ros::NodeHandle handle;
    ros::Subscriber joySub;
    ros::Publisher cmdPub;
    ros::ServiceClient client;
    geometry_msgs::Twist out;
    // keep the status of the switch that change the behaviour 
    // auto/drived of the robot, managed by the mux
    bool switchState;
            
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
public:
    void Prepare();
    void RunContinuously();
    void RunPeriodically();
};
