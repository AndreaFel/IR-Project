#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <sensor_msgs/LaserScan.h>
#include "polar_point.h"
#include <vector>

// Callback function called when a goal message is received
void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    // Implement actions upon receiving the goal here
    ROS_INFO("Received Goal: (%f, %f)", msg->goal.target_pose.pose.position.x, msg->goal.target_pose.pose.position.y);
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
            
        profile.push_back(PolarPoint( m.angle_min + (index * m.angle_increment),dist));
        prev = dist;
    }

    ROS_INFO("Profiles size: %d", profiles.size());

    // print profiles size and median point
    for(auto profile : profiles) {
        PolarPoint median = PolarPoint::getMedianPoint(profile);
        ROS_INFO("Profile size: %d, median: (d=%f,ar=%f)", profile.size(), median.getDistance(), median.getAngleRadians());
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

    // Keep spinning to receive and process incoming messages
    ros::spin();

    return 0;
}
