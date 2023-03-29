#ifndef _GAZEBO_FAULTY_GPS_PLUGIN_HH_
#define _GAZEBO_FAULTY_GPS_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/common.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/gzmath.hh"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/NavSatFix.h"

namespace gazebo {
    class GAZEBO_VISIBLE FaultyGPSPlugin : public SensorPlugin {
    enum State {NONE, LOSS_SIGNAL, MULTIPATH, CORRECT};
        public: FaultyGPSPlugin();
        public: virtual ~FaultyGPSPlugin();
        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
        protected: virtual void OnUpdate(sensors::GpsSensorPtr _sensor);
        protected: virtual void OnWorldUpdate(const common::UpdateInfo &_info);
    protected: void changeStatus();
    protected: void StartTimer();
        protected: sensors::GpsSensorPtr parentSensor;
        private: event::ConnectionPtr connection;
        private: event::ConnectionPtr updateConnection;
	private: ros::Subscriber sub;
	private: ros::Publisher pub;
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	private: float lossProb;
	private: float multiPathProb;
	private: gazebo::common::Timer timer;
	private: gazebo::common::Time timerDuration;
	private: State currentState;
	private: float latTranslation;
	private: float longTranslation;
	private: float maxLatTranslation;
	private: float maxLongTranslation;
    };
}
#endif
