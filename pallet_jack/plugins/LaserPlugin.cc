#include "LaserPlugin.hh"
// threshold when the lift is down
#define THRESHOLD 0.2
#define ORIENTATION_THRESHOLD 0.1
// threshold when the lift is up
#define THRESHOLD_UP 0.5
#define ORIENTATION_THRESHOLD_UP 1
#define MEASUREMENTS_NUMBER 5

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(LaserPlugin)

LaserPlugin::LaserPlugin():SensorPlugin(){
}

LaserPlugin::~LaserPlugin(){
}


void LaserPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    // init state
    state = false;
    // get parent sensor
    this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
    
    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
      gzerr << "ContactPlugin requires a ContactSensor.\n";
      return;
    }
    

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LaserPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
    
    // Init ROS and advertise a topic
    if(!ros::isInitialized()){
        // launch ros
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    
    // get ros node handler
    ros::NodeHandle node;
    
    publisher = node.advertise<std_msgs::Bool>("/pallet_jack/load", 1);
    
    std::cout << "executed load method from Laser Plugin\n\n";
}

void LaserPlugin::OnUpdate()
{
    // get the status of the father sensor, and find the most near object distance
    std::vector<double> ranges;
    // returns the array with all ranges
    parentSensor->Ranges(ranges);
    // if the object is centered the difference of ranges between the first and the last is 0
    double range_diff = ranges[0] -ranges[ranges.size()-1];
    int middleRange = ranges.size()/2;

      // add the measurements to the vector of measurements
      dist_measurements.push_back(ranges[middleRange]);
      ang_measurements.push_back(range_diff);
      // send msg only if collected enough data
      if(dist_measurements.size()>MEASUREMENTS_NUMBER){
	double dist_average = std::accumulate( dist_measurements.begin(), dist_measurements.end(), 0.0)/dist_measurements.size(); 
	double ang_avg = std::accumulate( ang_measurements.begin(), ang_measurements.end(), 0.0)/ang_measurements.size(); 
	// if the load is up the threshold are higher since we do not want to go down
	if(!state){
	    state = dist_average>parentSensor->RangeMin() && (dist_average < THRESHOLD) && (ang_avg<ORIENTATION_THRESHOLD);
	}
	else{
	  state = dist_average>parentSensor->RangeMin() && (dist_average < THRESHOLD_UP) && (ang_avg<ORIENTATION_THRESHOLD_UP);
	}
	// clear all measurements
	ang_measurements.clear();
	dist_measurements.clear();
      }    
    // send the msg on the topic
    std_msgs::Bool msg;
    msg.data = state;
    publisher.publish(msg);
}

bool LaserPlugin::isAcceptable(double range){
    return range>=0 && range<=parentSensor->RangeMax();
}
