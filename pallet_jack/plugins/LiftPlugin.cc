#include "LiftPlugin.hh"
#define TARGET_POS 0.5
#define THRESHOLD 0.01
#define SLEEP_TIME 1.5

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LiftPlugin)
  
LiftPlugin::LiftPlugin():ModelPlugin(){
}

void LiftPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // in Load method get the joint and subscribe to the topic
    //get joint
    this->prismaticJoint = _parent->GetJoint("chassis_lift_joint");
  
    // get lift
    this -> lift = _parent->GetLink("lift");
  
    // get model
    this -> model = _parent;
  
    // Init ROS and advertise a topic
    if(!ros::isInitialized()){
        // launch ros
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    
    // get ros node handler
    ros::NodeHandle node;
    
    //subscribe
    this-> subscriber = node.subscribe<std_msgs::Bool>("/pallet_jack/load", 1, &LiftPlugin::OnUpdate,this);
    
    std::cout<<"Load executed from Lift Plugin\n";
    
    this -> jointPid = common::PID(50000,1000000000,10000);
    
    this -> updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LiftPlugin::OnUpdateWorld,this,_1));
    
    this->model->GetJointController()->SetPositionPID(this->prismaticJoint->GetScopedName(),this->jointPid);
    
    // initialization
    this->lastMsg=false;
}

// On update method called when someone publishes on topic
void LiftPlugin::OnUpdate(const std_msgs::Bool::ConstPtr& msg)
{
    if(!this->lastMsg && msg->data){
        this->load_timestamp = ros::Time::now();
    }
    // save msg, need on the onUpdateWorld method
    this->lastMsg = msg->data;
}

void LiftPlugin::OnUpdateWorld(const common::UpdateInfo & /*_info*/)
{
    double last_load_interval = ros::Time::now().toSec() - this->load_timestamp.toSec(); 
    double jointPos = this->prismaticJoint->GetAngle(0).Radian();
    if(lastMsg && last_load_interval > SLEEP_TIME){
      // if the position is near the target lock the joint
	if(std::abs(jointPos-TARGET_POS)<THRESHOLD){
	    // lock the joint in the current position
	    this->prismaticJoint->SetLowStop(0,jointPos);
	    this->prismaticJoint->SetHighStop(0,jointPos);
	    // reset controller command
	    this->model->GetJointController()->Reset();
	}
	else{
	  this->model->GetJointController()->SetPositionTarget(this->prismaticJoint->GetScopedName(),TARGET_POS);
	  this->model->GetJointController()->Update();
	}
    }
    else{
	// reset positions
	this->prismaticJoint->SetLowStop(0,0);
	this->prismaticJoint->SetHighStop(0,TARGET_POS);
        this->jointPid.Reset();
        this->model->GetJointController()->Reset();
    }
}



LiftPlugin::~LiftPlugin()
{

}

