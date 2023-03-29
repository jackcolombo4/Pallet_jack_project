#include "ros/ros.h"
#include "turtlesim_actionlib/DrawPolygonAction.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <actionlib/server/simple_action_server.h>
#include <math.h>

typedef actionlib::SimpleActionServer<turtlesim_actionlib::DrawPolygonAction> SimpleActionServer;

typedef struct Poly{
    int side_length;
    float angle;
    int sides;
}Poly;

/* this class wraps a simple action server node */
class Server {
private:
    /* keep internally the work done */
    int sides_to_do;
    /* keep internaly the type of poly to draw */
    Poly polygon;
    /* keep internally the turtle pose */
    turtlesim::Pose turtle_pose;
    
    ros::NodeHandle handle;
    SimpleActionServer *delegated_sas;
    /* publish the commands to the turtle */
    ros::Publisher vel_pub;
    /* read the turtle pose topic */
    ros::Subscriber turtle_pose_sub;
    
    void executeCb(const turtlesim_actionlib::DrawPolygonGoalConstPtr& goal);
    void preemptCb();

    /* update the kept state of the turtle */
    void turtlePoseCallback(const turtlesim::Pose::ConstPtr& pose);
    
    bool correctlyDrawn(turtlesim::Pose& intial_position);
    void doAngle(float angle);
    bool doSide();
    void giveFeedback();
    void fail();
    void succeeded();
    void draw();
    void initWithCurrentPose(turtlesim::Pose& to_init);
public:
    Server();
    void Prepare();
};
