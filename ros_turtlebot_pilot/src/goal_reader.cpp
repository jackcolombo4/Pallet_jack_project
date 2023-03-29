#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#define NAME_OF_THIS_NODE "goal_reader"
#define PI 3.141592

int main(int argc, char** argv)
{
    float x, y, w;
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    ros::NodeHandle handler;
    ros::Publisher pub = handler.advertise<geometry_msgs::Pose>("/goal", 10);
    geometry_msgs::Pose msg;
    while (ros::ok()){
	std::cout << "\nEnter to give a goal\n";
	std::cin;
	std::cout << "x: ";
	std::cin >> x;
	std::cout << "y: ";
	std::cin >> y;
	std::cout << "Orientation (degree): ";
	std::cin >> w;
	msg.position.x = x;
	msg.position.y = y;
	// from degree to radians
	msg.orientation = tf::createQuaternionMsgFromYaw(w*PI/180);
	std::cout << msg;
	pub.publish(msg);
    }
}