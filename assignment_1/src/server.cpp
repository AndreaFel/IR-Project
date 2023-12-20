#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <assignment_1/detectionAction.h>


class DetectionAction {

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<assignment_1::DetectionAction> as_;
        std::string action_name_;
        assignment_1::DetectionFeedback feedback_;
        assignment_1::DetectionResult result_;

    public:
        DetectionAction(std::string name) : as_(nh_, name, boost::bind(&DetectionAction::executeCB, this, _1), false), action_name_(name){
            as_.start();
        }
        ~DetectionAction(void){}
        void executeCB(const assignment_1::DetectionGoalConstPtr &goal) {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		as_.setSucceeded(result_);
        }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "Detection");
    DetectionAction detection("Detection");
    ros::spin();
    return 0;
}

