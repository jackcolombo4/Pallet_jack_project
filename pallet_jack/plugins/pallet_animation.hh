#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <stdio.h>

class PalletAnimation {
private:

    ros::NodeHandle handle;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    geometry_msgs::Twist out;
            
    void laserCallback(const std_msgs::Bool::ConstPtr& msg);
public:
    void Prepare();
};
