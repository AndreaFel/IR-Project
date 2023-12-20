#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_1/DetectionAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_sender_node");
    ros::NodeHandle nh;
	actionlib::SimpleActionClient<assignment_1::DetectionAction> ac("Detection", true);
	
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started.");
	
    double n = 0;
    std::string input;
	assignment_1::DetectionGoal goal;
    while (ros::ok()) {
		
		ROS_INFO("You can write 'q' at any time to quit.");
		ROS_INFO("Insert point B coordinates and final orientation of the robot (in radiants):");
        
		// Read x-coordinate input
        std::cin >> input;
        // Exit the while loop if 'q' is entered
        if (input == "q") {
            ROS_INFO("Exiting...");
            break; 
        }

        // Set x-coordinate in the goal message
        n = std::stod(input);
		goal.goal.X = n;

        // Read y-coordinate input
        std::cin >> input;
        // Exit the while loop if 'q' is entered
        if (input == "q") {
            ROS_INFO("Exiting...");
            break;
        }

        // Set y-coordinate in the goal message
        n = std::stod(input);
		goal.goal.Y = n;

        // Read rotation input
        std::cin >> input;
        // Exit the while loop if 'q' is entered
        if (input == "q") {
            ROS_INFO("Exiting...");
            break;
        }

        // Set y-coordinate in the goal message
        n = std::stod(input);
		goal.goal.R = n;

        // Publish goal
        ac.sendGoal(goal);
        ROS_INFO("Goal sent!");
		
		// Wait for result
		ac.waitForResult(ros::Duration(0.0));
		
		// Check result
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}

    return 0;
}
