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
#include <vector>
#include <iostream>
#include <cmath>

//keep track of robot position
Position currentPosition = {};

// Map
nav_msgs::OccupancyGrid::ConstPtr wall_map = nullptr;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	wall_map = map;
	ROS_INFO("Map received");
}

void positionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
	double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
}

// Callback function called when a goal message is received
void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
	// Implement actions upon receiving the goal here
	ROS_INFO("Received Goal: (%f, %f)", msg->goal.target_pose.pose.position.x, msg->goal.target_pose.pose.position.y);
}

bool is_cylinder(std::vector<PolarPoint> profile) {
	PolarPoint median = PolarPoint::getMedianPoint(profile);
	ROS_INFO("Profile size: %d, median: (d=%f,ar=%f)", profile.size(), median.getDistance(), median.getAngleRadians());
	
	return false;
}

void laserCallback(const sensor_msgs::LaserScan& m){
	
	// From laser signal extract obstacle profiles with 50cm threshold
	std::vector<std::vector<PolarPoint>> profiles = Obstacle::getObstacleProfiles(m, 0.5);
	ROS_INFO("Profiles size: %d\n\n", profiles.size());
	
	// Obstacle identification
	std::vector<Obstacle> obstacles;
	for(auto profile : profiles) 
		obstacles.push_back(Obstacle(profile, currentPosition));

	// print obstacles
	for(auto obstacle : obstacles) {
		/*
		ROS_INFO("Obstacle [%s]: (x=%f, y=%f, r=%f)", 
		    (obstacle.getShape() == Obstacle::Shape::Cylinder ? "Cylinder" : (obstacle.getShape() == Obstacle::Shape::Wall ? "Wall" : "Unknown")),
		    obstacle.getCenter().getX(), 
		    obstacle.getCenter().getY(), 
		    obstacle.getRadius()
		);
		//*/
		if(obstacle.getShape() == Obstacle::Shape::Cylinder) {
		    ROS_INFO("Obstacle [%s]: (x=%f, y=%f, r=%f)", 
		        "Cylinder",
		        obstacle.getCenter().getX(), 
		        obstacle.getCenter().getY(), 
		        obstacle.getRadius()
		    );
		}
	}


}

class DetectionAction {
	protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_1::DetectionAction> as_;
        std::string action_name_;
        assignment_1::DetectionFeedback feedback_;
        assignment_1::DetectionResult result_;

    public:

        DetectionAction(std::string name, ros::NodeHandle nh) : as_(nh, name, boost::bind(&DetectionAction::executeCB, this, _1), false), action_name_(name){
            as_.start();
        }

        ~DetectionAction(void){}

        void executeCB(const assignment_1::DetectionGoalConstPtr &goal) {
			//send position to robot
			ros::Publisher goal_publisher = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
			ros::Duration(1.0).sleep();

			// Create a MoveBaseActionGoal message to store goal information
			move_base_msgs::MoveBaseActionGoal goal_msg;
			goal_msg.header.frame_id = "map";
			goal_msg.goal.target_pose.header.frame_id = "map";
			goal_msg.goal.target_pose.pose.position.z = sin(goal->goal.R);
			goal_msg.goal.target_pose.pose.orientation.x = 0.0;
			goal_msg.goal.target_pose.pose.orientation.y = 0.0;
			goal_msg.goal.target_pose.pose.orientation.z = 0.0;
			goal_msg.goal.target_pose.pose.orientation.w = cos(goal->goal.R);
			goal_msg.goal.target_pose.pose.position.x = goal->goal.X;
			goal_msg.goal.target_pose.pose.position.y = goal->goal.Y;
			goal_publisher.publish(goal_msg);
			
			ROS_INFO("Goal sent to robot.");
			
			ros::topic::waitForMessage<move_base_msgs::MoveBaseActionResult>("/move_base/result", nh_);
			as_.setSucceeded(result_);
        }
        
        
};

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_receiver_node");
	ros::NodeHandle nh;
	
	// get map
	ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);

	// Subscribe to the "/move_base/goal" topic to receive goal messages
	ros::Subscriber goal_subscriber = nh.subscribe("/move_base/goal", 10, goalCallback);

	// Subscribe to the "/scan_raw" topic to receive laser scan messages
	ros::Subscriber laser_sub = nh.subscribe("scan_raw", 1000, laserCallback);
	
	// Subscribe to the "/move_base/feedback" topic to receive feedback messages
	ros::Subscriber feedback_subscriber = nh.subscribe("/move_base/feedback", 10, positionCallback);

	//get position from client
	DetectionAction Detection("Detection", nh);
	
	ros::spin();
	
    return 0;
}
