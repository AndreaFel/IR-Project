#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main(int argc, char **argv) {
    
    // // Initialize ROS node
    // ros::init(argc, argv, "goal_sender_node");
    // ros::NodeHandle nh;

    // // Create a publisher to send goal messages
    // ros::Publisher goal_publisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    // // Wait for the publisher to register
    // ros::Duration(1.0).sleep();

    // // Create a MoveBaseActionGoal message to store goal information
    // move_base_msgs::MoveBaseActionGoal goal_msg;
    // goal_msg.header.frame_id = "map";
    // goal_msg.goal.target_pose.header.frame_id = "map";
    // goal_msg.goal.target_pose.pose.position.z = 0.0;
    // goal_msg.goal.target_pose.pose.orientation.x = 0.0;
    // goal_msg.goal.target_pose.pose.orientation.y = 0.0;
    // goal_msg.goal.target_pose.pose.orientation.z = 0.0;
    // goal_msg.goal.target_pose.pose.orientation.w = 1.0;

    // double n = 0;
    // std::string input;
    // while (ros::ok()) {

    //     // Read x-coordinate input
    //     std::cin >> input;
    //     // Exit the while loop if 'q' is entered
    //     if (input == "q") {
    //         ROS_INFO("Exiting...");
    //         break; 
    //     }

    //     // Set x-coordinate in the goal message
    //     n = std::stod(input);
    //     goal_msg.goal.target_pose.pose.position.x = n;

    //     // Read y-coordinate input
    //     std::cin >> input;
    //     // Exit the while loop if 'q' is entered
    //     if (input == "q") {
    //         ROS_INFO("Exiting...");
    //         break;
    //     }

    //     // Set y-coordinate in the goal message
    //     n = std::stod(input);
    //     goal_msg.goal.target_pose.pose.position.y = n;

    //     // Publish goal
    //     goal_publisher.publish(goal_msg);
    //     ROS_INFO("Goal sent!");
		
	// 	// Wait for result
	// 	move_base_msgs::MoveBaseActionResult::ConstPtr received_msg;
	// 	received_msg = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionResult>("/move_base/result", nh);
		
	// 	// Check result
	// 	if(received_msg->status.status==3)
	// 		ROS_INFO("Movement completed!");
	// 	else
	// 		ROS_INFO("Movement failed!");
		
	// 	// Perform a 360-degree rotation (Currently commented out)
	// 	ROS_INFO("Starting rotation...");
		
    //     int status = 0; // To check that every action goes well
    //     int counter = 1; // Rotate of 120° for 3 times
        
	// 	do{
	// 		// Define rotation of counter*120° degrees as quaternion
	// 		tf2::Quaternion quat_rotation;
	// 		quat_rotation.setRPY(0, 0, counter*2*M_PI/3);
	// 		goal_msg.goal.target_pose.pose.orientation.x = quat_rotation.x();
	// 		goal_msg.goal.target_pose.pose.orientation.y = quat_rotation.y();
	// 		goal_msg.goal.target_pose.pose.orientation.z = quat_rotation.z();
	// 		goal_msg.goal.target_pose.pose.orientation.w = quat_rotation.w();
			
	// 		// Publish goal
	// 		goal_publisher.publish(goal_msg);
			
	// 		// Wait for result before sending the next one
	// 		received_msg = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionResult>("/move_base/result", nh);
			
	// 		// Update status and counter
	// 		status = received_msg->status.status;
	// 		counter++;
			
	// 	}while(status==3 && counter<=3);
		
	// 	// Check rotation status
	// 	if(status==3)
	// 		ROS_INFO("Rotation completed!");
	// 	else
	// 		ROS_INFO("Rotation failed!");
	// }

    actionlib::SimpleActionClient<assignment_1::DetectionAction> ac("Detection", true);
    ROS_INFO("Waiting for action server to start.");

    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    assignment_1::DetectionGoal goal;
    goal.order = 20;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else ROS_INFO("Action did not finish before the time out.");

    return 0;
}