#include "detection_action.h"

/** 
 * @brief Callback function for the "/move_base/result" topic subscriber
 * 
 * This function is called every time a new message is published on the "/move_base/result" topic.
 * It sets the send_feedback variable to false, so that the feedback loop is stopped.
 * 
 * @param msg Result message
 */
void resultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
	send_feedback = false;
}

/**
 * @brief Constructor for the detection action server
 * 
 * @param name Name of the action server
 * @param nh ROS node handle
 */
DetectionAction::DetectionAction(std::string name, ros::NodeHandle nh) : as_(nh, name, boost::bind(&DetectionAction::executeCB, this, _1), false), action_name_(name){
    as_.start();
}

/**
 * @brief Callback function for the detection action server
 * 
 * This function is called every time a new goal is received by the action server.
 * It sends the robot to the goal position and detects the cylinders in the environment.
 * It sends feedback messages to the client with the current position of the robot and
 * the detected cylinders. It also sends a result message when the robot reaches the goal position.
 * 
 * @param goal Goal message
 */
void DetectionAction::executeCB(const assignment_1::DetectionGoalConstPtr &goal) 
{
    // Subscribe to the "/move_base/result" topic to receive result messages
	ros::Subscriber result_subscriber = nh.subscribe("/move_base/result", 1, resultCallback);

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
    
    //wait for robot to reach goal
    while(send_feedback){

        // get current position
        feedback_.position.X = currentPosition.getPoint().getX();
        feedback_.position.Y = currentPosition.getPoint().getY();
        feedback_.position.R = currentPosition.getOrientation();
        
        // get current cylinders
        feedback_.cylinders.current.clear();
        for(auto cyl: currentCylinders){
            assignment_1::coordinate coord;
            coord.X = cyl.getCenter().getX();
            coord.Y = cyl.getCenter().getY();
            feedback_.cylinders.current.push_back(coord);
        }
        
        // get max cylinders
        feedback_.cylinders.maximum.clear();
        for(auto cyl: maxCylinders){
            assignment_1::coordinate coord;
            coord.X = cyl.getCenter().getX();
            coord.Y = cyl.getCenter().getY();
            feedback_.cylinders.maximum.push_back(coord);
        }
        
        // check if goal is cancelled
        if (as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }
        
        // publish feedback
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