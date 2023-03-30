#ifndef _GAZEBO_LASER_PLUGIN_HH
#define _GAZEBO_LASER_PLUGIN_HH

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
	// \brief A plugin that reads from laser and publishes on a topic
	// \brief A plugin that reads from laser and publishes on a topic
	class LaserPlugin : public SensorPlugin
{
	// Constructor
public: LaserPlugin();
public: virtual ~LaserPlugin();

public: virtual void Load(sensors::SensorPtr _sensor,sdf::ElementPtr _sdf);

private: virtual void OnUpdate();

private: sensors::RaySensorPtr parentSensor;

private: event::ConnectionPtr updateConnection;

private: ros::Publisher publisher;

// true if the load is up, false otherwise
private: bool state;
// vectors containing measurements, they are averaged to avoid false positive
std::vector<double> dist_measurements;
std::vector<double> ang_measurements;
private: bool isAcceptable(double range);

};
}
#endif
