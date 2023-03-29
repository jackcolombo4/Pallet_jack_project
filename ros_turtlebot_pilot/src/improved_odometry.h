#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>

class ImprovedOdometry {
    typedef void(ImprovedOdometry::*OdomMethod)(float, float);
    enum Mode {EULER, RUNGE_KUTTA, EXACT};
private:
    ros::NodeHandle handler;
    ros::Publisher odomPub;
    ros::Subscriber jointStateSub;
    
    // number of data that are read, averaged, and then used to 
    // compute new odom average should reduce the error impact
    int dataNumber;
    std::vector<float> linearVelocities;
    std::vector<float> angularVelocities; 
    int currentIteration;
    geometry_msgs::Pose2D currentPosition;

    Mode mode;
    // contains string to be read as param and corresponding value of enum mode
    std::map<std::string, Mode> modeMap;
    // contains mode and corresponding method to be called
    std::map<Mode, OdomMethod> methodMap;
    
    void mapInit();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    ros::Time lastTS;
    
    void odomEuler(float linearVel, float angularVel);
    void odomRungeKutta(float linearVel, float angularVel);
    void odomExact(float linearVel, float angularVel);
    
public:
    void Prepare();
    void Loop();
};