#pragma once

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
#include "position.h"
#include "detection_action.h"
#include <vector>
#include <iostream>
#include <cmath>

send_feedback = true;

/**
 * @brief detection action server class
 * 
 * This class implements the detection action server. It is used to send the robot to a given position
 * and detect the cylinders in the environment. The action server sends feedback messages to the client
 * with the current position of the robot and the detected cylinders. It also sends a result message
 * when the robot reaches the goal position.
 */
class DetectionAction {
private:
    ros::NodeHandle nh_;                                                // ROS node handle
    actionlib::SimpleActionServer<assignment_1::DetectionAction> as_;   // action server
    std::string action_name_;                                           // action name  
    assignment_1::DetectionFeedback feedback_;                          // feedback message                   
    assignment_1::DetectionResult result_;                              // result message
    
    
public:
    /**
     * @brief Constructor for the detection action server
     * 
     * @param name Name of the action server
     * @param nh ROS node handle
     */
    DetectionAction(std::string name, ros::NodeHandle nh);

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
    void executeCB(const assignment_1::DetectionGoalConstPtr &goal);
             
};