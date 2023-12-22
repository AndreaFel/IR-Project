#include "ros/ros.h"
#include "detection_action.h"

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

	//Instantiate the DetectionAction class
	DetectionAction Detection("Detection", nh);
	
	ros::spin();
	
    return 0;
}
