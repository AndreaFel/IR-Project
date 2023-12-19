#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_1/detectionAction.h>

int main(int argc, char **argv) {

    actionlib::SimpleActionClient<assignment_1::detectionAction> ac("detection", true);
    ROS_INFO("Waiting for action server to start.");

    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    assignment_1::detectionGoal goal;
    goal.goal.X = 20;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else ROS_INFO("Action did not finish before the time out.");

    return 0;
}
