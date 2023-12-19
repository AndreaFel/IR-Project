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

#include <actionlib/server/simple_action_server.h>
#include <iassignment_1/DetectionAction.h>

//keep track of robot position
Position currentPosition = {};

// Map
nav_msgs::OccupancyGrid::ConstPtr wall_map = nullptr;

void positionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
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
    
    // From laser signal extract obstacle profiles with 50cm threshold
    std::vector<std::vector<PolarPoint>> profiles = Obstacle::getObstacleProfiles(m, 0.5);
    ROS_INFO("Profiles size: %d\n\n", profiles.size());
    
    // Obstacle identification
    std::vector<Obstacle> obstacles;
    for(auto profile : profiles) 
        obstacles.push_back(Obstacle(profile, currentPosition));

    // print obstacles
    for(auto obstacle : obstacles) {
        /*ROS_INFO("Obstacle [%s]: (x=%f, y=%f, r=%f)", 
            (obstacle.getShape() == Obstacle::Shape::Cylinder ? "Cylinder" : (obstacle.getShape() == Obstacle::Shape::Wall ? "Wall" : "Unknown")),
            obstacle.getCenter().getX(), 
            obstacle.getCenter().getY(), 
            obstacle.getRadius()
        );*/
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

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    wall_map = map;
    ROS_INFO("Map received");
}


class DetectionAction {

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_1::DetectionAction> as_;
        std::string action_name_;
        assignment_1::DetectionFeedback feedback_;
        assignment_1::DetectionResult result_;

    public:
        DetectionAction(std::string name) : as_(nh_, name, boost::bind(&DetectionAction::executeCB, this, _1), false), action_name_(name){
            as_.start();
        }
        ~DetectionAction(void){}
        void executeCB(const assignment_1::DetectionGoalConstPtr &goal) {

            ros::Rate r(1);
            bool success = true;
            feedback_.sequence.clear();
            feedback_.sequence.push_back(0);
            feedback_.sequence.push_back(1);

            for(int i=1; i<=goal->order; i++) {
                if (as_.isPreemptRequested() || !ros::ok()) {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    as_.setPreempted();
                    success = false;
                    break;
                }

                feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
                as_.publishFeedback(feedback_);
                r.sleep();
            }

            if(success) {
                result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
        }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "Detection");
    DetectionAction Detection("Detection");
    ros::spin();
    return 0;
}

// int main(int argc, char **argv) {
    
//     // Initialize ROS node
//     ros::init(argc, argv, "goal_receiver_node");
//     ros::NodeHandle nh;

//     // get map
//     ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);

//     // Subscribe to the "/move_base/goal" topic to receive goal messages
//     ros::Subscriber goal_subscriber = nh.subscribe("/move_base/goal", 10, goalCallback);

//     // Subscribe to the "/scan_raw" topic to receive laser scan messages
//     ros::Subscriber laser_sub = nh.subscribe("scan_raw", 1000, laserCallback);
    
//     // Subscribe to the "/move_base/feedback" topic to receive feedback messages
//     ros::Subscriber feedback_subscriber = nh.subscribe("/move_base/feedback", 10, positionCallback);

//     // Keep spinning to receive and process incoming messages
//     ros::spin();

//     return 0;
// }
