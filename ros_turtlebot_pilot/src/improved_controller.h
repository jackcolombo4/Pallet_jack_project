#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include "ros_turtlebot_pilot/GetMode.h"

class ImprovedController {
    //enumerate the different behaviour of the robot
    enum CurrentAction {IDLE, INITIAL_ROTATION, MOVING_FORWARD, FINAL_ROTATION};
    // enumerate the possible rotation 
	enum Rotation{CLOCKWISE,COUNTERWISE};
    
    typedef void(ImprovedController::*MovementMethod)();
    
private:
    ros::NodeHandle handle;
    ros::ServiceClient client; // get the modality
    
    //command to publish
    geometry_msgs::Twist cmd;
    
    // read the state of the robot
    ros::Subscriber odometrySub;
    
    // read the target position
    ros::Subscriber goalSub;
    
    // publish the velocity command for the robot
    ros::Publisher cmdPub;
    
    // enable and disable the PIDs
    ros::Publisher angularPidEnabler;
    ros::Publisher linearPidEnabler;
    
    // publish the state of the robot on the PIDs state topic
    ros::Publisher angularStatePidPub;
    ros::Publisher linearStatePidPub;
    
    // publish the setpoint on the PIDs state topic
    ros::Publisher angularSetpointPidPub;
    ros::Publisher linearSetpointPidPub;
    
    // read the pid output
    ros::Subscriber angularEffortPidReader;
    ros::Subscriber linearEffortPidReader;
    
    CurrentAction currentAction;
    geometry_msgs::Pose2D currentPosition;
    geometry_msgs::Pose2D goalPosition;
    bool goal_reached; // true if goal is reached
    bool is_auto; // true if auto mode
	
    // angular velocity of the robot for controlling rotation
    double angularVel;
    // linear velocity of the robot for controlling rotation
    double linearVel;
    float minDist; //min distance reached from the goal
    float currentDistance;
    
    //map containing the different action methods
    std::map<CurrentAction, MovementMethod> methodMap;
	void initMap();
    
    void initialRotation();
    void moving();
    void finalRotation();
    void idle();

    void resetPid(ros::Publisher pidPub);
    void shutDownPid(ros::Publisher pidPub);
    
    //callback methods
    void goalCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void pidLinearEffortCallback(const std_msgs::Float64::ConstPtr& msg);
    void pidAngularEffortCallback(const std_msgs::Float64::ConstPtr& msg);
    
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
    // choose the direction in which the robot should rotate
    Rotation chooseDirection(double currentRotation,double direction);
    
    double computeSetPointForRotation(double direction,Rotation rotation);
   
    
public:
    void Prepare();
    void publish();
    void getModality();
};