#include "ros/ros.h"
#include "assignment_1/msg1.h"
#include <sstream>
#include <map>
#include <string>


int main(int argc, char **argv) {

    ros::init(argc, argv, "client");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<assignment_1::msg1>("message", 1000);
    ros::Rate loop_rate(5); // Set the loop rate to 5Hz

    while (ros::ok()) {

        assignment_1::msg1 msg;

        msg.X = 0;
        msg.Y = 1;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}