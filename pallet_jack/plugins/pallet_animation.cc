#include "pallet_animation.hh"
#define NAME_OF_THIS_NODE "pallet_animation"


void PalletAnimation::laserCallback(const std_msgs::Bool::ConstPtr& msg)
{
   // create a twist msg 
  out = geometry_msgs::Twist();
  // the msg is false if the load is not correctly positioned 
  if(!msg->data){
    //with negative linear velocity in order to move near the box
    out.linear.x = -1;
    publisher.publish(out);
  }
  else{
    // stop and quit
    publisher.publish(out);  
    ros::shutdown();
  }
}

void PalletAnimation::Prepare()
{
  subscriber = handle.subscribe("/pallet_jack/load",1,&PalletAnimation::laserCallback,this);
  publisher = handle.advertise<geometry_msgs::Twist>("pallet_jack/cmd_vel",1);
  
}

int main(int argc, char** argv){
  ros::init(argc,argv,NAME_OF_THIS_NODE);
  PalletAnimation anim;
  anim.Prepare();
  ros::spin();
  return 1;
}