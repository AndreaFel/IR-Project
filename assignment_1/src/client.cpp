#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Function prototype for rotating 360 degrees
void rotate360Degrees(ros::Publisher& publisher) {
    // TODO: move here the code for the Rotation of 360 degrees
}

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_sender_node");
    ros::NodeHandle nh;

    // Create a publisher to send goal messages
    ros::Publisher goal_publisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    // Wait for the publisher to register
    ros::Duration(1.0).sleep();

    // Create a MoveBaseActionGoal message to store goal information
    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.goal.target_pose.header.frame_id = "map";
    goal_msg.goal.target_pose.pose.position.z = 0.0;
    goal_msg.goal.target_pose.pose.orientation.x = 0.0;
    goal_msg.goal.target_pose.pose.orientation.y = 0.0;
    goal_msg.goal.target_pose.pose.orientation.z = 0.0;
    goal_msg.goal.target_pose.pose.orientation.w = 1.0;

    double n = 0;
    std::string input;
    while (ros::ok()) {

        // Read x-coordinate input
        std::cin >> input;
        // Exit the while loop if 'q' is entered
        if (input == "q") {
            ROS_INFO("Exiting...");
            break; 
        }

        // Set x-coordinate in the goal message
        n = std::stod(input);
        goal_msg.goal.target_pose.pose.position.x = n;

        // Read y-coordinate input
        std::cin >> input;
        // Exit the while loop if 'q' is entered
        if (input == "q") {
            ROS_INFO("Exiting...");
            break;
        }

        // Set y-coordinate in the goal message
        n = std::stod(input);
        goal_msg.goal.target_pose.pose.position.y = n;

        // Publish goal
        goal_publisher.publish(goal_msg);
        ROS_INFO("Goal sent!");

        // Perform a 360-degree rotation (Currently commented out)
        // rotate360Degrees(goal_publisher);

        // Set quaternion for 360-degree rotation
        tf2::Quaternion quat;
        // 6.28319 radians = 360 degrees
        quat.setRPY(0, 0, 6.28319);
        tf2::convert(quat, goal_msg.goal.target_pose.pose.orientation);

        // Publish rotation goal
        goal_publisher.publish(goal_msg);
        ROS_INFO("Rotation sent!");
    }

    return 0;
}

