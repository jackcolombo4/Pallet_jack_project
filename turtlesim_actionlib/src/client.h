#include "ros/ros.h"
#include "draw_figures_msgs/Draw.h"
#include "turtlesim_actionlib/DrawPolygonAction.h"
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<turtlesim_actionlib::DrawPolygonAction> SimpleActionClient;

class Client {
private:
    ros::NodeHandle handle;
    ros::Subscriber draw_subscriber;
    SimpleActionClient *delegated_sac;
    void drawCallback(const draw_figures_msgs::Draw::ConstPtr& msg);
    void doneCb(const actionlib::SimpleClientGoalState& state, const turtlesim_actionlib::DrawPolygonResult::ConstPtr& result);
    void activeCb();
    void feedbackCb(const turtlesim_actionlib::DrawPolygonFeedback::ConstPtr& feedback);
public:
    Client();
    void Prepare();
};
