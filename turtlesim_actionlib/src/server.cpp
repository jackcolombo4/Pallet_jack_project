#include "server.h"

#define PI 3.14159265358979323846
#define GO_STRAIGHT_SPEED 0.5
#define ROTATE_SPEED 0.1
#define CHECK_MARGIN 0.01

Server::Server() {

}

void Server::Prepare(){
    vel_pub = handle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    turtle_pose_sub = handle.subscribe("turtle1/pose", 10, &Server::turtlePoseCallback, this);
    delegated_sas = new SimpleActionServer(this -> handle, "draw_polygon", boost::bind(&Server::executeCb, this, _1), false);
    delegated_sas->registerPreemptCallback(boost::bind(&Server::preemptCb, this));
    delegated_sas -> start();
    ROS_INFO("SERVER: server ready\n");
}

void Server::turtlePoseCallback(const turtlesim::Pose::ConstPtr& pose)
{
    turtle_pose.x = pose -> x;
    turtle_pose.y = pose -> y;
    turtle_pose.theta = pose -> theta;
    turtle_pose.linear_velocity = pose -> linear_velocity;
    turtle_pose.angular_velocity = pose -> angular_velocity;
}

/*
 * called when a goal is received, check if valid input, save the goal,
 * then start the draw algorithm
 */
void Server::executeCb(const turtlesim_actionlib::DrawPolygonGoalConstPtr& goal) 
{
    ROS_INFO("SERVER: received goal:\nlength: %d\nsides: %d", goal -> length, goal -> sides);
    if (goal -> length <= 0 || goal -> sides < 3) { // fail, not possible
        fail();
    } else {
        // if input is ok save internally goal
        polygon.side_length = goal -> length;
        polygon.sides = goal -> sides;
        polygon.angle = PI - (goal -> sides - 2) * PI / goal -> sides;
        sides_to_do = goal -> sides;
        // then start the draw algorithm
        draw();
    }
}

void Server::preemptCb()
{
    ROS_INFO("SERVER: goal preempted\n");
    delegated_sas->setPreempted();
}


/*
 * draw algorithm, core of the application
 */
void Server::draw()
{
    while (sides_to_do > 0) {
        if (delegated_sas->isPreemptRequested()) {
            return;
        }
        bool successfully_drawn_side = doSide(); // draw a side and get the outcome
        if (!successfully_drawn_side) {
            fail();
            return;
        }
        sides_to_do--;
        giveFeedback();
        if (sides_to_do != 0) {
            doAngle(polygon.angle);
        }else { // after the last side place the turtle horizontal
            doAngle(2*PI - turtle_pose.theta);
        }
    }
    succeeded();
}

/*
 * get in input the initial position, then from the internal state 
 * of the server object read the lenght of the side and the current 
 * position of the turtle to chek if the side have been correctl drawn
 */
bool Server::correctlyDrawn(turtlesim::Pose& initial_position)
{
    float distance = sqrt(pow(turtle_pose.x - initial_position.x, 2) +
        pow(turtle_pose.y - initial_position.y, 2));
    if (abs(distance - polygon.side_length) < CHECK_MARGIN) return true;
    return false;
}

/*
 * open loop, send twist msgs with velocity 
 */
void Server::doAngle(float angle)
{
    ros::Time t0 = ros::Time::now();
    ros::Time t1;
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = ROTATE_SPEED;
    float current_rotation = 0;
    while (current_rotation < angle) {
        vel_pub.publish(vel_msg);
        t1 = ros::Time::now();
        current_rotation = (t1.toSec()-t0.toSec())*ROTATE_SPEED;
    }
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}

/*
 * "open" loop, send twist msgs with velocity
 * after each command check that the presumed 
 * distance computed internally is equal to the 
 * actually current distance, if not return 
 * false (the action has failed) 
 */
bool Server::doSide()
{
    ros::Time t0 = ros::Time::now();
    ros::Time t1;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = GO_STRAIGHT_SPEED;
    float presumed_distance = 0;
    turtlesim::Pose side_start_pose;
    initWithCurrentPose(side_start_pose);
    while (presumed_distance < polygon.side_length) {
        vel_pub.publish(vel_msg);
        t1 = ros::Time::now();
        presumed_distance = (t1.toSec()-t0.toSec())*GO_STRAIGHT_SPEED;
        float real_distance = 
            sqrt(pow(turtle_pose.x - side_start_pose.x, 2) + pow(turtle_pose.y - side_start_pose.y, 2));
        if (abs(real_distance - presumed_distance) > CHECK_MARGIN) return false;
    }
    vel_msg.linear.x = 0;
    vel_pub.publish(vel_msg);
    return true;
}

/*
 * send via the simple action server a feedback containing
 * the number of angle that have been drawn
 */
void Server::giveFeedback()
{
    turtlesim_actionlib::DrawPolygonFeedback feedback;
    feedback.drawn_sides = polygon.sides - sides_to_do;
    delegated_sas->publishFeedback(feedback);
}

/*
 * send failed via the simple action server
 */
void Server::fail()
{
    turtlesim_actionlib::DrawPolygonResult result; 
    result.succesful = false;
    delegated_sas -> setAborted(result);
}

/*
 * send succeeded via the simple action server 
 */
void Server::succeeded()
{
    turtlesim_actionlib::DrawPolygonResult result; 
    result.succesful = true;
    delegated_sas -> setSucceeded(result);
}


/*
 * use the turtle pose saved in this object to init the 
 * input turtle pose variable
 */
void Server::initWithCurrentPose(turtlesim::Pose& to_init){
    to_init.x = this -> turtle_pose.x;
    to_init.y = this -> turtle_pose.y;
    to_init.theta = this -> turtle_pose.theta;
    to_init.linear_velocity = this -> turtle_pose.linear_velocity;
    to_init.angular_velocity = this -> turtle_pose.angular_velocity;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "draw_polygon_server");
    Server server;
    server.Prepare();
    ros::spin();
    return 0;
}
