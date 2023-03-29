#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include "ros_turtlebot_pilot/GetMode.h"

class Controller {
    //enumerate the different behaviour of the robot
    enum CurrentAction {IDLE, INITIAL_ROTATION, MOVING_FORWARD, FINAL_ROTATION};
    // enumerate the possible rotation 
	enum Rotation{CLOCKWISE,COUNTERWISE};
    
private:
    ros::NodeHandle handle;
    ros::ServiceClient client; // get the modality
        
    // read the state of the robot
    ros::Subscriber odometrySub;
    
    // read the target position
    ros::Subscriber goalSub;
    
    // publish the velocity command for the robot
    ros::Publisher cmdPub;
    
    // enable and disable the PID dinstance controller
    ros::Publisher pidEnabler;
    
    // publish the state of the robot on the pid state topic
    ros::Publisher statePidPub;
    
    // publish the position of the goal on the pid state topic
    // as distance from (0, 0) in abs coordinates
    ros::Publisher setpointPidPub;
    
    // read the pid output
    ros::Subscriber effortPidReader;
    
    CurrentAction currentAction;
    geometry_msgs::Pose2D currentPosition;
    geometry_msgs::Pose2D goalPosition;
    bool goal_reached; // true if goal is reached
    bool is_auto; // true if auto mode
	
	// angular velocity of the robot for controlling rotation
	double angularVel;
	// linear velocity of the robot for controlling rotation
	double linearVel;
	/*
     * minimum distance from the goal in a path reached from the 
     * robot. When the current distance becomes greather that minDist
     * the robot is going away from the goal
     */
	double minDist;
    
    void resetPid();
    void shutDownPid();
    
    //callback methods
    void goalCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void pidEffortCallback(const std_msgs::Float64::ConstPtr& msg);
    
    //compute if it is time to change behavior
    bool nearEnough();
    bool initialRotateEnough();
    bool finalRotateEnough();
    
    // utility methods
    geometry_msgs::Pose2D convertIn2DPose(const nav_msgs::Odometry::ConstPtr& odom);
    geometry_msgs::Pose2D convertIn2DPose(const geometry_msgs::Pose::ConstPtr&  pose);
    float distance (geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2);
    double computeDirectionAngle();

    // given the current rotation and the rotation the robot want to reach, 
    // choose the direction in which teh robot should rotate
    Rotation chooseDirection(double currentRotation,double direction);
    
	double computeSetPointForRotation(double currentRotation,double direction,Rotation rotation);
    
public:
    Controller();
    void Prepare();
    void getModality();
};