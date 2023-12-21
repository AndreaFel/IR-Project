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

//maximum number of cylinders ever detected
std::vector<Obstacle> maxCylinders, currentCylinders;
bool send_feedback = true;

void positionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
	double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
}

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

void resultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
	send_feedback = false;
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
        	maxCylinders.clear();
        	ros::Rate r(5);
        	
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
			
			//send feedback
			send_feedback = true;
			bool success = true;
			
			while(send_feedback){
				feedback_.position.X = currentPosition.getPoint().getX();
		    	feedback_.position.Y = currentPosition.getPoint().getY();
		    	feedback_.position.R = currentPosition.getOrientation();
		    	
		    	feedback_.cylinders.current.clear();
		    	for(auto cyl: currentCylinders){
			    	assignment_1::coordinate coord;
			    	coord.X = cyl.getCenter().getX();
			        coord.Y = cyl.getCenter().getY();
			    	feedback_.cylinders.current.push_back(coord);
			    }
			    
			    feedback_.cylinders.maximum.clear();
		    	for(auto cyl: maxCylinders){
			    	assignment_1::coordinate coord;
			    	coord.X = cyl.getCenter().getX();
			        coord.Y = cyl.getCenter().getY();
			    	feedback_.cylinders.maximum.push_back(coord);
			    }
			    
			    if (as_.isPreemptRequested() || !ros::ok()){
					ROS_INFO("%s: Preempted", action_name_.c_str());
					as_.setPreempted();
					success = false;
					break;
				}
				
				as_.publishFeedback(feedback_);
				r.sleep();
			}
			
			//send result
			if(success){
				result_.cylinders.maximum.clear();
				result_.cylinders.current.clear();

				for(auto obstacle : maxCylinders) {
					
					assignment_1::coordinate newCoordinate;
					newCoordinate.X = obstacle.getCenter().getX();
					newCoordinate.Y = obstacle.getCenter().getY();
					
					result_.cylinders.maximum.push_back(newCoordinate);
				}

				for(auto obstacle : currentCylinders) {
					assignment_1::coordinate newCoordinate;
					newCoordinate.X = obstacle.getCenter().getX();
					newCoordinate.Y = obstacle.getCenter().getY();
					
					result_.cylinders.current.push_back(newCoordinate);
				}

				as_.setSucceeded(result_);
			}
        }
        
        
};

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_receiver_node");
	ros::NodeHandle nh;

	// Subscribe to the "/scan_raw" topic to receive laser scan messages
	ros::Subscriber laser_sub = nh.subscribe("scan_raw", 1000, laserCallback);
	
	// Subscribe to the "/move_base/feedback" topic to receive feedback messages
	ros::Subscriber feedback_subscriber = nh.subscribe("/move_base/feedback", 10, positionCallback);
	
	// Subscribe to the "/move_base/result" topic to receive result messages
	ros::Subscriber result_subscriber = nh.subscribe("/move_base/result", 1, resultCallback);

	//get position from client
	DetectionAction Detection("Detection", nh);
	
	ros::spin();
	
    return 0;
}
