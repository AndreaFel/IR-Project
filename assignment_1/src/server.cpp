#include "ros/ros.h"
#include "assignment_1/msg1.h"

void messageCallback(const assignment_1::msg1::ConstPtr& msg) {

    ROS_INFO("Coordinate in X is: [%d] Coordinate in Y is: [%d]", msg->X, msg->Y);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("message", 1000, messageCallback);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}