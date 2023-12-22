#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_1/DetectionAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/**
 * @brief Feedback callback function for the detection action client
 * 
 * This function is called every time a new feedback message is received from the action server.
 * It prints the current position of the robot and the detected cylinders.
 * 
 * @param feedback feedback message received from the action server
 */
void feedbackCallback(const assignment_1::DetectionFeedbackConstPtr& feedback) {

    ROS_INFO("Position - X: %f, Y: %f, R: %f",
             feedback->position.X, feedback->position.Y, feedback->position.R);

    ROS_INFO("Number of coordinates in cylinders.current: %lu", feedback->cylinders.current.size());

    for (const auto& coord : feedback->cylinders.current) {
        ROS_INFO("Coordinates in cylinders.current - X: %f, Y: %f", coord.X, coord.Y);
    }

    ROS_INFO("Number of coordinates in cylinders.maximum: %lu", feedback->cylinders.maximum.size());

    for (const auto& coord : feedback->cylinders.maximum) {
        ROS_INFO("Coordinates in cylinders.maximum - X: %f, Y: %f", coord.X, coord.Y);
    }
}

// Mandatory callback functions for the feedback
void doneCb(const actionlib::SimpleClientGoalState& state, const assignment_1::DetectionResultConstPtr& feedback) {}
void activeCb() {}


/**
 * @brief Main function for the goal sender node
 * 
 * This function initializes the ROS node and the action client.
 * It asks the user to insert the goal position and orientation of the robot.
 * It asks the user if they want real-time feedback.
 * It sends the goal to the action server and waits for the result.
 * 
 * @param argc number of arguments
 * @param argv array of arguments
 * @return int exit code
 */
int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_sender_node");
    ros::NodeHandle nh;
	actionlib::SimpleActionClient<assignment_1::DetectionAction> ac("Detection", true);

    //ac.registerFeedbackCallback(feedbackCallback);
	
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
        std::cout<<"Insert x coordinate: (q to quit) : "<<std::endl;
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
        std::cout<<"Insert y coordinate: (q to quit) : "<<std::endl;
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

        // Set rotation in the goal message
        n = std::stod(input);
		goal.goal.R = n;

        // Ask user if they want feedback
        ROS_INFO("Do you want real-time feedback? (Enter 's' for yes or 'n' for no):");
        std::cin >> input;

        if (input == "q") {
            ROS_INFO("Exiting...");
            break;

        } else if (input == "s") {
            // User wants real-time feedback
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCallback);
            ROS_INFO("Real-time feedback started. Goal sent!");

            // Publish goal
            // ac.sendGoal(goal);
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCallback);

        } else if (input == "n") {
            // User does not want real-time feedback
            ac.sendGoal(goal);

        } else {
            ROS_INFO("Invalid input. Please enter 's' for real-time feedback or 'n' for no feedback.");
        }

        ROS_INFO("Goal sent!");
            
        // Wait for result
        ac.waitForResult(ros::Duration(0.0));
        
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The action was completed successfully");
            const assignment_1::DetectionResultConstPtr& result = ac.getResult();

            ROS_INFO("\nTotal number of cylinders seen in the path: %lu", result->cylinders.maximum.size());

            // Stampa le coordinate dei cilindri visti lungo il percorso
            for (const auto& element : result->cylinders.maximum) {
                ROS_INFO("Position of the cylinder seen along the path - X: %f, Y: %f", element.X, element.Y);
            }

            ROS_INFO("\nTotal number of cylinders currently being viewed: %lu", result->cylinders.current.size());

            // Stampa le coordinate dei cilindri attualmente in vista
            for (const auto& element : result->cylinders.current) {
                ROS_INFO("Position of the cylinder currently being viewed - X: %f, Y: %f", element.X, element.Y);
            }
            
        } else {
            ROS_ERROR("The action was not completed successfully");
        }

	}

    return 0;
}

