#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"

#include "polar_point.h"
#include "cartesian_point.h"
#include "obstacle.h"
#include "position.h"
#include <vector>
#include <iostream>

Position currentPosition;

void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
    double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
	
	std::cout<<currentPosition<<std::endl;
}

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_receiver_node");
    ros::NodeHandle nh;
	
	currentPosition = {};

    // Subscribe to the "/move_base/feedback" topic to receive feedback messages
    ros::Subscriber goal_subscriber = nh.subscribe("/move_base/feedback", 10, feedbackCallback);

    ros::spin();

    return 0;
}
