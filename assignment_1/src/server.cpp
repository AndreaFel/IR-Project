#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

// Callback function called when a goal message is received
void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    // Implement actions upon receiving the goal here
    ROS_INFO("Received Goal: (%f, %f)", msg->goal.target_pose.pose.position.x, msg->goal.target_pose.pose.position.y);
}

int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "goal_receiver_node");
    ros::NodeHandle nh;

    // Subscribe to the "/move_base/goal" topic to receive goal messages
    ros::Subscriber goal_subscriber = nh.subscribe("/move_base/goal", 10, goalCallback);

    // Keep spinning to receive and process incoming messages
    ros::spin();

    return 0;
}
