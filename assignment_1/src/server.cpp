#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"
#include <actionlib/server/simple_action_server.h>
#include <assignment_1/DetectionAction.h>

#include "polar_point.h"
#include "cartesian_point.h"
#include "obstacle.h"
#include "position.h"
#include "detection_action.h"
#include <vector>
#include <iostream>
#include <cmath>

//keep track of robot position
Position currentPosition = {};

//maximum number of cylinders ever detected
std::vector<Obstacle> maxCylinders, currentCylinders;

/**
 * @brief Callback function for the "/move_base/feedback" topic subscriber
 * 
 * This function is called every time a new message is published on the "/move_base/feedback" topic.
 * It extracts the robot position from the feedback message and stores it in the currentPosition variable.
 * 
 * @param m Feedback message
 */
void positionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
	double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
}

/**
 * @brief Callback function for the "/scan_raw" topic subscriber
 * 
 * This function is called every time a new message is published on the "/scan_raw" topic.
 * It extracts the obstacle profiles from the laser scan message and prints them.
 * To detect the obstacle profiles, it uses the Obstacle::getObstacleProfiles() function
 * and a threshold of 50cm. If the signal make a jump of more than 50cm, it is considered
 * a new obstacle profile. The Obstacle class constructor is used to identify the shape
 * of the obstacle profile.
 * 
 * @param m Laser scan message
 */
void laserCallback(const sensor_msgs::LaserScan& m){
	
	// From laser signal extract obstacle profiles with 50cm threshold
	std::vector<std::vector<PolarPoint>> profiles = Obstacle::getObstacleProfiles(m, 0.5);
	ROS_INFO("Profiles size: %ld\n\n", profiles.size());
	
	// Obstacle identification
	std::vector<Obstacle> obstacles;
	for(auto profile : profiles) 
		obstacles.push_back(Obstacle(profile, currentPosition));

	// print obstacles
	currentCylinders.clear();
	for(auto obstacle : obstacles) {
		if(obstacle.getShape() == Obstacle::Shape::Cylinder) {
		    ROS_INFO("Obstacle [%s]: (x=%f, y=%f, r=%f)", 
		        "Cylinder",
		        obstacle.getCenter().getX(), 
		        obstacle.getCenter().getY(), 
		        obstacle.getRadius()
		    );
			currentCylinders.push_back(obstacle);
		}
	}
	
	if(currentCylinders.size() > maxCylinders.size()){
		maxCylinders.clear();
		for(auto obstacle : obstacles) 
			if(obstacle.getShape() == Obstacle::Shape::Cylinder) 
				maxCylinders.push_back(obstacle);
				
	}
}

/**
 * @brief Main function for the goal receiver node
 * 
 * This function initializes the ROS node and subscribes to the "/scan_raw" topic to receive
 * laser scan messages. It also subscribes to the "/move_base/feedback" topic to receive
 * feedback messages. It creates a DetectionAction object to send the robot to a given position
 * and detect the cylinders in the environment. The action server sends feedback messages to the client
 * with the current position of the robot and the detected cylinders. It also sends a result message
 * when the robot reaches the goal position.
 * 
 * @param argc unused
 * @param argv unused
 * @return int exit code
 */
int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_receiver_node");
	ros::NodeHandle nh;

	// Subscribe to the "/scan_raw" topic to receive laser scan messages
	ros::Subscriber laser_sub = nh.subscribe("scan_raw", 1000, laserCallback);
	
	// Subscribe to the "/move_base/feedback" topic to receive feedback messages
	ros::Subscriber feedback_subscriber = nh.subscribe("/move_base/feedback", 10, positionCallback);

	//get position from client
	DetectionAction Detection("Detection", nh);
	
	ros::spin();
	
    return 0;
}
