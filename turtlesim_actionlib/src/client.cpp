#include "client.h"

#define NAME_OF_THIS_NODE "client"

Client::Client()
{

}

void Client::Prepare() {
    draw_subscriber = handle.subscribe("draw", 1, &Client::drawCallback, this);
    this -> delegated_sac = new SimpleActionClient("draw_polygon", true);
    ROS_INFO("CLIENT: waiting for server\n");
    this -> delegated_sac -> waitForServer();
    ROS_INFO("CLIENT: connected, client ready");
}

// Called once when the goal completes
void Client::doneCb(const actionlib::SimpleClientGoalState& state, const turtlesim_actionlib::DrawPolygonResult::ConstPtr& result) {
    ROS_INFO("CLIENT: finished in state [%s]", state.toString().c_str());
    ROS_INFO("CLIENT: answer: %s", result->succesful ? "Successful drawn" : "Unsuccess, found obstacle or invalid polygon");
}

// Called once when the goal becomes active
void Client::activeCb() {
    ROS_INFO("CLIENT: goal just went active");
}

// Called every time feedback is received for the goal
void Client::feedbackCb(const turtlesim_actionlib::DrawPolygonFeedback::ConstPtr& feedback) {
    ROS_INFO("CLIENT: %d have been drawn", feedback->drawn_sides);
}

void Client::drawCallback(const draw_figures_msgs::Draw::ConstPtr& msg){
    ROS_INFO("CLIENT: sending the goal:\nlength = %d\nsides = %d", msg->length, msg->sides);
    turtlesim_actionlib::DrawPolygonGoal goal;
    goal.length = msg -> length;
    goal.sides = msg -> sides;
    delegated_sac -> sendGoal(goal,
                        boost::bind(&Client::doneCb, this, _1, _2),
                        boost::bind(&Client::activeCb, this),
                        boost::bind(&Client::feedbackCb, this, _1));
    ROS_INFO("CLIENT: goal sent\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    Client client;
    client.Prepare();
    ros::spin();
    return (0);
}
