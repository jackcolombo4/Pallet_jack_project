#include "controller.h"

#define NAME_OF_THIS_NODE "controller"

#define RATE 1000

#define POS_THRESHOLD 0.05
#define ORIENTATION_THRESHOLD 0.001 
#define SPEED_THRESHOLD 0.0001
#define DIST_THRESHOLD 0.1

#define PI 3.141592

Controller::Controller()
{

}

void Controller::Prepare()
{
    // read from the parameter server which odom topic to use
    ros::NodeHandle localHandler("~");
    std::string which_odom;
    localHandler.param<std::string>("odom", which_odom, /*default*/ "odom");
    
    odometrySub = handle.subscribe(which_odom, 1, &Controller::odometryCallback, this);
    goalSub = handle.subscribe("goal", 1, &Controller::goalCallback, this);
    effortPidReader = handle.subscribe("control_effort", 1, &Controller::pidEffortCallback, this);
    
    cmdPub = handle.advertise<geometry_msgs::Twist>("cmd_auto", 10);
    pidEnabler = handle.advertise<std_msgs::Bool>("pid_enable", 1);
    setpointPidPub = handle.advertise<std_msgs::Float64>("setpoint", 1);
    statePidPub = handle.advertise<std_msgs::Float64>("state", 1);
    
    //wait until the mux node is running
    ros::service::waitForService("getMode",-1);
    // obtain the service client
    client = handle.serviceClient<ros_turtlebot_pilot::GetMode>("getMode");
    // get the modality
    getModality();
    this->goal_reached = true;
    // set default state	
    this->currentAction = IDLE;
}

/*
 * Call the service, saves the Modality, if not auto changes state
 */
void Controller::getModality()
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

/*
 * called when a new goal is published for the robot. It is time to
 * rotate in order to aim to the goal.
 */
void Controller::goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    this->goalPosition = convertIn2DPose(msg);
    
    // enable the pid controller
    std_msgs::Bool e_msg;
    e_msg.data = true;
    pidEnabler.publish(e_msg);
}

/*
 * each time that the position of the robot is recomputed, the setpoint 
 * of the pid and the behavior of the robot need to be updated
 */
void Controller::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // save the position and the ang vel in the internal state
    currentPosition = convertIn2DPose(msg);
    angularVel = msg->twist.twist.angular.z;
    linearVel = msg->twist.twist.linear.x;
    //needed to understand if the robot is goig away from the robot
    double currentDistance = distance(goalPosition,currentPosition);
	
    // msgs for the pid
    std_msgs::Float64 setpoint;
    std_msgs::Float64 state;
    /*
     * change behavior when an action has finished,
     * otherwise publish the setpoint and the state
     * on the pid topic
     */
    switch (currentAction){
      
      // if we are in this state with a goal this means that we have changed from manual to auto without having reached a goal
      case IDLE:
	  // if the goal is not reached and the modality is auto change the state, we need to reach the goal
	  if(!goal_reached && this->is_auto){
	      this->currentAction = INITIAL_ROTATION;
	  }
	  break;
        
        case MOVING_FORWARD:
	    if(nearEnough()){ // stop going straight
			currentAction = FINAL_ROTATION;
			/* print the status */
			std::cout << "STATUS: " << this->currentAction << "\n";
			resetPid();
	    }else{ // actuate
			/*
             * if we are going away from the goal return to initial
             * rotation state in order to recompute the direction
             * and re-aim to the goal
             */
			if(currentDistance>minDist+DIST_THRESHOLD){
				currentAction=INITIAL_ROTATION;
				/* print the status */
				std::cout << "STATUS: " << this->currentAction << "\n";
				resetPid();
			}
			else{
				setpoint.data = currentDistance;
				state.data = 0;
			}
	    }
	    break;
	    
	case INITIAL_ROTATION:
        /*
         * the condition std::abs(angularVel)<=SPEED_THRESHOLD make possible to have oscillation
         * around the setpoint direction. No need to rotate too slow, faster reaching of setpoint
         */
	    if(initialRotateEnough() && std::abs(angularVel)<=SPEED_THRESHOLD){ //stop rotating
			// init lastDistance
			minDist=currentDistance;
			currentAction = MOVING_FORWARD;
			/* print the status */
			std::cout << "STATUS: " << this->currentAction << "\n";
			resetPid();
	    }else{ // actuate rotation
			double direction = computeDirectionAngle();
			Rotation rotation = chooseDirection(currentPosition.theta,direction);
			setpoint.data = computeSetPointForRotation(currentPosition.theta,direction,rotation);
			state.data = 0;
	    }
	    break;
	    
	case FINAL_ROTATION:
        // as intial rotation, only the setpoint changes
	    if(finalRotateEnough() && std::abs(angularVel)<SPEED_THRESHOLD){
			currentAction = IDLE;
			/* print the status */
			std::cout << "GOAL REACHED SUCCESFULLY! \n\n ";
			// set goal reached
			this->goal_reached = true;
			shutDownPid();
	    }else{
			Rotation rotation = chooseDirection(currentPosition.theta,goalPosition.theta);
			setpoint.data = computeSetPointForRotation(currentPosition.theta,goalPosition.theta,rotation);
			state.data = 0;
	    }
	    break;
    }
    
    // check if need to update mindist, the robot nears the goal
    if(currentDistance<minDist-DIST_THRESHOLD)
		minDist=currentDistance;
    
	if(currentAction!=IDLE){    
		statePidPub.publish(state);
		setpointPidPub.publish(setpoint);
	}
	
}

void Controller::pidEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
    geometry_msgs::Twist cmd;
    
    //actuate
    switch(currentAction) {
	case INITIAL_ROTATION:
	case FINAL_ROTATION:
	    cmd.angular.z = msg->data;
	    break;
	    
	case MOVING_FORWARD:
	    cmd.linear.x = msg->data;
	    break;
    }
    if (currentAction != IDLE){
		cmdPub.publish(cmd);
    }
}

bool Controller::nearEnough()
{
   return distance(currentPosition, goalPosition) < POS_THRESHOLD;
}

bool Controller::finalRotateEnough()
{
    // when the goal position is near zero it's ok also to have an orientation near 2PI (second check)
    return (std::abs(currentPosition.theta - goalPosition.theta) < ORIENTATION_THRESHOLD
	    || std::abs(2*PI - currentPosition.theta - goalPosition.theta) < ORIENTATION_THRESHOLD);
}

bool Controller::initialRotateEnough()
{
    // when the orientation is near direction
    return (std::abs(currentPosition.theta-computeDirectionAngle()) < ORIENTATION_THRESHOLD
    // when orientation is around 2PI and direction is near 0 or when the orienation is near 0 and direction near 2PI
	    || std::abs(2*PI - currentPosition.theta - computeDirectionAngle()) < ORIENTATION_THRESHOLD);
}

double Controller::computeDirectionAngle()
{
    double angle = std::atan((goalPosition.y-currentPosition.y)/(goalPosition.x-currentPosition.x));
    if (goalPosition.x < currentPosition.x && goalPosition.y < currentPosition.y || //2 and 4 quadrant
	goalPosition.x < currentPosition.x && goalPosition.y > currentPosition.y)
	angle += PI;
    if (goalPosition.x > currentPosition.x && goalPosition.y < currentPosition.y) //3 quadrant
	angle += 2*PI;
    return angle;
}

Controller::Rotation Controller::chooseDirection(double currentRotation, double direction)
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

double Controller::computeSetPointForRotation(double currentRotation, double direction, Controller::Rotation rotation)
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

 

void Controller::resetPid()
{
    std_msgs::Bool msg;
    msg.data = false;
    pidEnabler.publish(msg);
    msg.data = true;
    pidEnabler.publish(msg);
}

void Controller::shutDownPid()
{
    std_msgs::Bool msg;
    msg.data = false;
    pidEnabler.publish(msg);
}



geometry_msgs::Pose2D Controller::convertIn2DPose(const nav_msgs::Odometry::ConstPtr& odom)
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

geometry_msgs::Pose2D Controller::convertIn2DPose(const geometry_msgs::Pose::ConstPtr& pose)
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

float Controller::distance(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2)
{
    return std::sqrt(std::pow(pose1.x - pose2.x,2) + std::pow(pose1.y - pose2.y,2));
}


int main(int argc, char** argv) {
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    Controller node;
   
    node.Prepare();
  
    ros::Rate r(RATE);
    
    while(ros::ok()){
        ros::spinOnce();
	node.getModality();
        r.sleep();
    }
  
  return (0);
}
