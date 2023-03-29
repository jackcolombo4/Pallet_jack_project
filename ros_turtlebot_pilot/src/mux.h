#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros_turtlebot_pilot/SetMode.h"
#include "ros_turtlebot_pilot/GetMode.h"

class Mux {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber joySub;
    ros::Subscriber autoSub;
    ros::Publisher cmdPub;
    ros::ServiceServer getModeSr;
    ros::ServiceServer modeSr;
    
    geometry_msgs::Twist command;
    int mode; // 1 is auto, 0 is driven
        
    void joyCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void autoCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool modeService(ros_turtlebot_pilot::SetMode::Request &req, ros_turtlebot_pilot::SetMode::Response &res);
    bool getMode(ros_turtlebot_pilot::GetMode::Request &req, ros_turtlebot_pilot::GetMode::Response &res);
public:    
    void Prepare();
    void RunContinuously();
    void RunPeriodically();
};
