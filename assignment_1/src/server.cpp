#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"

#include "polar_point.h"
#include "cartesian_point.h"
#include "obstacle.h"
#include "position.h"
#include <vector>
#include <iostream>

//keep track of robot position
Position currentPosition = {};

void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
    double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
	
	//un-comment to debug
	//std::cout<<currentPosition<<std::endl;
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
    std::vector<std::vector<PolarPoint>> profiles;
	std::vector<PolarPoint> profile;
    int index = 0;    
    double threshold = 0.5;
    double prev = m.ranges.front();

    for(auto dist : m.ranges) {
        if (abs(dist - prev) > threshold) {
            //ROS_INFO("EOP dist: %f, prev: %f, diff: %f ", dist, prev, abs(dist - prev));
            profiles.push_back(profile);
            profile.clear();
        }
            
        profile.push_back(PolarPoint( m.angle_min + (index++ * m.angle_increment),dist));
        prev = dist;
    }

    ROS_INFO("Profiles size: %d\n\n", profiles.size());

    // print profiles size and median point
    for(auto profile : profiles) {
        PolarPoint median = PolarPoint::getMedianPoint(profile);
        //ROS_INFO("Profile size: %d, median: (d=%f,ar=%f)", profile.size(), median.getDistance(), median.getAngleRadians());

        Obstacle obstacle(profile);

    }

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    // Access map data
    int width = map->info.width;
    int height = map->info.height;
    std::vector<int8_t> mapData = map->data;

    // Assuming obstacle threshold (adjust as needed)
    int obstacleThreshold = 50;

    // Simple obstacle identification
    for (int i = 0; i < width * height; ++i) {
        // Check if the cell is considered an obstacle based on threshold
        if (mapData[i] >= obstacleThreshold) {
            // Calculate cell indices in 2D grid
            int x = i % width;
            int y = i / width;

            // Print or process identified obstacles (for demonstration)
            ROS_INFO("Obstacle found at cell (%d, %d)", x, y);
            // Here, you can perform further processing or marking of identified obstacles
        }
    }
}

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_receiver_node");
    ros::NodeHandle nh;

    // Subscribe to the "/move_base/goal" topic to receive goal messages
    ros::Subscriber goal_subscriber = nh.subscribe("/move_base/goal", 10, goalCallback);

    // Subscribe to the "/scan_raw" topic to receive laser scan messages
    ros::Subscriber sub = nh.subscribe("scan_raw", 1000, laserCallback);
    
    // Subscribe to the "/move_base/feedback" topic to receive feedback messages
    ros::Subscriber goal_subscriber = nh.subscribe("/move_base/feedback", 10, feedbackCallback);

    //ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);

    // Keep spinning to receive and process incoming messages
    ros::spin();

    return 0;
}
