#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <assignment_1/detectionAction.h>


class detectionAction {

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_1::detectionAction> as_;
        std::string action_name_;
        assignment_1::detectionFeedback feedback_;
        assignment_1::detectionResult result_;

    public:
        detectionAction(std::string name) : as_(nh_, name, boost::bind(&detectionAction::executeCB, this, _1), false), action_name_(name){
            as_.start();
        }
        ~detectionAction(void){}
        void executeCB(const assignment_1::detectionGoalConstPtr &goal) {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		as_.setSucceeded(result_);
        }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "detection");
    detectionAction detection("detection");
    ros::spin();
    return 0;
}

