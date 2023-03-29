#include "improved_controller.h"

#define RATE 100

#define NAME_OF_THIS_NODE "improved_controller"
#define POS_THRESHOLD 0.05
#define ORIENTATION_THRESHOLD 0.001 
#define INITIAL_ORIENTATION_THRESHOLD 0.1
#define INITIAL_SPEED_THRESHOLD 0.1
#define SPEED_THRESHOLD 0.0001
#define DIST_THRESHOLD 0.1
#define PI 3.141592

void ImprovedController::Prepare()
{
    initMap();
    
    // read from the parameter server which odom topic to use
    ros::NodeHandle localHandler("~");
    std::string which_odom;
    localHandler.param<std::string>("odom", which_odom, /*default*/ "odom");
    
    odometrySub = handle.subscribe(which_odom, 1, &ImprovedController::odometryCallback, this);
    goalSub = handle.subscribe("goal", 1, &ImprovedController::goalCallback, this);
    
    linearEffortPidReader = handle.subscribe("linear_control_effort", 1, &ImprovedController::pidLinearEffortCallback, this);
    angularEffortPidReader = handle.subscribe("angular_control_effort", 1, &ImprovedController::pidAngularEffortCallback, this);
    
    cmdPub = handle.advertise<geometry_msgs::Twist>("cmd_auto", 10);
    
    linearPidEnabler = handle.advertise<std_msgs::Bool>("linear_pid_enable", 1);
    angularPidEnabler = handle.advertise<std_msgs::Bool>("angular_pid_enable", 1);
    
    linearSetpointPidPub = handle.advertise<std_msgs::Float64>("linear_setpoint", 1);
    angularSetpointPidPub = handle.advertise<std_msgs::Float64>("angular_setpoint", 1);
    
    linearStatePidPub = handle.advertise<std_msgs::Float64>("linear_state", 1);
    angularStatePidPub = handle.advertise<std_msgs::Float64>("angular_state", 1);
    
    //wait until the mux node is running
    ros::service::waitForService("getMode",-1);
    // obtain the service client
    client = handle.serviceClient<ros_turtlebot_pilot::GetMode>("getMode");
    getModality();
    
    this->currentAction = IDLE;
    this->goal_reached=true;
}

/*
 * Call the service, saves the Modality, if not auto changes state
 */
void ImprovedController::getModality()
{
    ros_turtlebot_pilot::GetMode msg;
    client.call(msg);
    int mode = msg.response.mode;
    if(mode==1)
      is_auto=true;
    else is_auto=false;
    
    if(!is_auto)
      this->currentAction = IDLE;
}


void ImprovedController::initMap()
{
    methodMap[CurrentAction::FINAL_ROTATION] = &ImprovedController::finalRotation;
    methodMap[CurrentAction::MOVING_FORWARD] = &ImprovedController::moving;
    methodMap[CurrentAction::INITIAL_ROTATION] = &ImprovedController::initialRotation;
    methodMap[CurrentAction::IDLE]=&ImprovedController::idle;
}

void ImprovedController::goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    this->goalPosition = convertIn2DPose(msg);
    this-> goal_reached = false;
    
    // enable the angular pid controller
    std_msgs::Bool e_msg;
    e_msg.data = true;
    angularPidEnabler.publish(e_msg);
    linearPidEnabler.publish(e_msg);
}

void ImprovedController::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // save the position and the ang vel in the internal state
    currentPosition = convertIn2DPose(msg);
    angularVel = msg->twist.twist.angular.z;
    linearVel = msg->twist.twist.linear.x;
    //needed to understand if the robot is goig away from the robot
    currentDistance = distance(goalPosition,currentPosition);
    /*
     * change behavior when an action has finished,
     * otherwise publish the setpoint and the state
     * on the pid topic
     */
    (this->*methodMap.at(currentAction))();
    
    /*
     * IF NEED TO CORRECT
    if(currentDistance>minDist+DIST_THRESHOLD){
        currentAction=INITIAL_ROTATION;
        /* print the status *//*
        std::cout << "STATUS: " << this->currentAction << "\n";
        resetPid();
    }
    // check if need to update mindist, the robot nears the goal
    if(currentDistance<minDist-DIST_THRESHOLD)
        minDist=currentDistance;
    */
}

void ImprovedController::initialRotation()
{
    // msgs for the pid
    std_msgs::Float64 setpoint;
    std_msgs::Float64 state;
    /*
     * the condition std::abs(angularVel)<=SPEED_THRESHOLD make possible to have oscillation
     * around the setpoint direction. No need to rotate too slow, faster reaching of setpoint
     */
    if(initialRotateEnough() && std::abs(angularVel)<=INITIAL_SPEED_THRESHOLD){ //stop rotating
        // init lastDistance
        minDist=currentDistance;
        currentAction = MOVING_FORWARD;
        /* print the status */
        std::cout << "STATUS: " << this->currentAction << "\n";
        resetPid(angularPidEnabler);
    }else{ // actuate rotation
        double direction = computeDirectionAngle();
        Rotation rotation = chooseDirection(currentPosition.theta,direction);
        setpoint.data = computeSetPointForRotation(direction,rotation);
        state.data = 0;
        angularSetpointPidPub.publish(setpoint);
        angularStatePidPub.publish(state);
    }
}

void ImprovedController::finalRotation()
{
    // msgs for the pid
    std_msgs::Float64 setpoint;
    std_msgs::Float64 state;
    // as intial rotation, only the setpoint changes
    if(finalRotateEnough() && std::abs(angularVel)<SPEED_THRESHOLD){
        currentAction = IDLE;
	goal_reached = true;
        /* print the status */
        std::cout << "GOAL REACHED SUCCESFULLY! \n\n ";
        shutDownPid(angularPidEnabler);
    }else{
        Rotation rotation = chooseDirection(currentPosition.theta,goalPosition.theta);
        setpoint.data = computeSetPointForRotation(goalPosition.theta,rotation);
        state.data = 0;
        angularSetpointPidPub.publish(setpoint);
        angularStatePidPub.publish(state);
    }
}

void ImprovedController::moving()
{
    // msgs for the pid
    std_msgs::Float64 angularSetpoint;
    std_msgs::Float64 angularState;
    std_msgs::Float64 linearSetpoint;
    std_msgs::Float64 linearState;
    if(nearEnough()){ // stop going straight
        currentAction = FINAL_ROTATION;
        /* print the status */
        std::cout << "STATUS: " << this->currentAction << "\n";
        resetPid(linearPidEnabler);
        resetPid(angularPidEnabler);
    }else{
        double angle = computeDirectionAngle();
        Rotation rotation = chooseDirection(currentPosition.theta, angle);
        angularState.data=0;
        linearState.data=0;
        angularSetpoint.data=computeSetPointForRotation(angle, rotation);
        linearSetpoint.data = currentDistance;
        //publish
        linearSetpointPidPub.publish(linearSetpoint);
        linearStatePidPub.publish(linearState);
        angularSetpointPidPub.publish(angularSetpoint);
        angularStatePidPub.publish(angularState);
    }
}

void ImprovedController::idle()
{
  // if the goal is not reached and the modality is auto change the state, we need to reach the goal
    if(!goal_reached && this->is_auto){
      this->currentAction = INITIAL_ROTATION;
    }
}


void ImprovedController::pidAngularEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
    //actuate
    cmd.angular.z = msg->data;
}

void ImprovedController::pidLinearEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
    //actuate
    cmd.linear.x = msg->data;
   
}

/*
 * UTILITY METHODS
*/

bool ImprovedController::nearEnough()
{
   return distance(currentPosition, goalPosition) < POS_THRESHOLD;
}

bool ImprovedController::finalRotateEnough()
{
    // when the goal position is near zero it's ok also to have an orientation near 2PI (second check)
    return (std::abs(currentPosition.theta - goalPosition.theta) < ORIENTATION_THRESHOLD
        || std::abs(2*PI - currentPosition.theta - goalPosition.theta) < ORIENTATION_THRESHOLD);
}

bool ImprovedController::initialRotateEnough()
{
    // when the orientation is near direction
    return (std::abs(currentPosition.theta-computeDirectionAngle()) < INITIAL_ORIENTATION_THRESHOLD
    // when orientation is around 2PI and direction is near 0 or when the orienation is near 0 and direction near 2PI
        || std::abs(2*PI - currentPosition.theta - computeDirectionAngle()) < INITIAL_ORIENTATION_THRESHOLD);
}

double ImprovedController::computeDirectionAngle()
{
    double angle = std::atan((goalPosition.y-currentPosition.y)/(goalPosition.x-currentPosition.x));
    if (goalPosition.x < currentPosition.x && goalPosition.y < currentPosition.y || //2 and 4 quadrant
    goalPosition.x < currentPosition.x && goalPosition.y > currentPosition.y)
    angle += PI;
    if (goalPosition.x > currentPosition.x && goalPosition.y < currentPosition.y) //3 quadrant
    angle += 2*PI;
    return angle;
}

ImprovedController::Rotation ImprovedController::chooseDirection(double currentRotation, double direction)
{
    if(0<=currentRotation && currentRotation<PI){
        if(direction<currentRotation+PI && direction>currentRotation){
            return Rotation::COUNTERWISE;
        }
        return Rotation::CLOCKWISE;
    }
    else{
        if(direction>currentRotation || (direction>=0 && direction<(PI-(2*PI-currentRotation)))){
            return Rotation::COUNTERWISE;
        }
        return Rotation::CLOCKWISE;
    }
}

double ImprovedController::computeSetPointForRotation(double direction, ImprovedController::Rotation rotation)
{
    double setpoint;
    if(rotation == CLOCKWISE){
            if(direction-currentPosition.theta<0){
                setpoint= direction-currentPosition.theta;
            }else{
                setpoint= -(currentPosition.theta+2*PI-direction);
            }
        }
        else{
            if(direction-currentPosition.theta<0){
                setpoint = 2*PI-currentPosition.theta + direction;
            }
            else{
                setpoint = direction-currentPosition.theta;
            }
        }
    return setpoint;
}


geometry_msgs::Pose2D ImprovedController::convertIn2DPose(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::Pose2D pose2D;
    pose2D.x = odom->pose.pose.position.x;
    pose2D.y = odom->pose.pose.position.y;
    geometry_msgs::Pose pose = odom->pose.pose;
    // convert quaternion in yaw
    double yaw = tf::getYaw(pose.orientation);
    if(yaw<0)
        yaw +=2*PI;
    pose2D.theta = yaw;
    return pose2D;
}

geometry_msgs::Pose2D ImprovedController::convertIn2DPose(const geometry_msgs::Pose::ConstPtr& pose)
{
    geometry_msgs::Pose2D pose2D;
    pose2D.x = pose->position.x;
    pose2D.y = pose->position.y;
    // conversion between grade and randiant
    double yaw = tf::getYaw(pose->orientation);
    if(yaw<0)
        yaw +=2*PI;
    pose2D.theta = yaw;
    return pose2D;
}

float ImprovedController::distance(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2)
{
    return std::sqrt(std::pow(pose1.x - pose2.x,2) + std::pow(pose1.y - pose2.y,2));
}


void ImprovedController::resetPid(ros::Publisher pidEnabler)
{
    std_msgs::Bool msg;
    msg.data = false;
    pidEnabler.publish(msg);
    msg.data = true;
    pidEnabler.publish(msg);
}

void ImprovedController::shutDownPid(ros::Publisher pidEnabler)
{
    std_msgs::Bool msg;
    msg.data = false;
    pidEnabler.publish(msg);
}

void ImprovedController::publish(){
  // publish only if current action is not idle
   if (currentAction != IDLE && (cmd.angular.z!=0 || cmd.linear.x!=0)){
        cmdPub.publish(cmd);
    }
    cmd.angular.z=0;
    cmd.linear.x=0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    ImprovedController node;
   
    node.Prepare();
  
    ros::Rate r(RATE);
    
    while(ros::ok()){
        ros::spinOnce();
	node.getModality();
	node.publish();
        r.sleep();
    }
  
  return (0);
}
