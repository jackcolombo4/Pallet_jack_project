#ifndef _JOYPAD_SIMULATOR
#define _JOYPAD_SIMULATOR

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <ncurses.h>
#include <cstdlib>
#include <map>
#include <tuple>

class JoyPadSimulator{
private: ros::NodeHandle nodeHandle;
	ros::Publisher pub;
	sensor_msgs::Joy msg_out;
	struct termios oldt,newt;
	// button map key is the ascii code of the char, value is the position in button array
	std::map<int,int> buttonMap;
	// key is the keypressed, value is a tuple <linearVel,angularVel>
	std::map<int,std::vector <int>> arrowMap;
	void printMenu();
	///\brief setup terminal in raw mode disabling echo
	void setupTerminal();
	///\brief restore old settings
	void restoreTerminal();
	// initialize a msg with the correct number of cells in array
	sensor_msgs::Joy initMsg();

	
public: 
  void Prepare();
  void Run();
  
};

#endif