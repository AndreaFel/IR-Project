#include "detection_action.h"

//global variables for callback usage only *************************************

//keep track of the feedback loop
bool send_feedback = true;

//keep track of robot position
Position currentPosition = {};

//maximum number of cylinders ever detected
std::vector<Obstacle> maxCylinders, currentCylinders;

//callback functions **********************************************************

/**
 * @brief Callback function for the "/move_base/feedback" topic subscriber
 * 
 * This function is called every time a new message is published on the "/move_base/feedback" topic.
 * It extracts the robot position from the feedback message and stores it in the currentPosition variable.
 * 
 * @param m Feedback message
 */
void positionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
	double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
}

/**
 * @brief Callback function for the "/scan_raw" topic subscriber
 * 
 * This function is called every time a new message is published on the "/scan_raw" topic.
 * It extracts the obstacle profiles from the laser scan message and prints them.
 * To detect the obstacle profiles, it uses the Obstacle::getObstacleProfiles() function
 * and a threshold of 50cm. If the signal make a jump of more than 50cm, it is considered
 * a new obstacle profile. The Obstacle class constructor is used to identify the shape
 * of the obstacle profile.
 * 
 * @param m Laser scan message
 */
void laserCallback(const sensor_msgs::LaserScan& m){
	
	// From laser signal extract obstacle profiles with 50cm threshold
	std::vector<std::vector<PolarPoint>> profiles = Obstacle::getObstacleProfiles(m, 0.5);
	ROS_INFO("Profiles size: %ld\n\n", profiles.size());
	
	// Obstacle identification
	std::vector<Obstacle> obstacles;
	for(auto profile : profiles) 
		obstacles.push_back(Obstacle(profile, currentPosition));

	// print obstacles
	currentCylinders.clear();
	for(auto obstacle : obstacles) {
		if(obstacle.getShape() == Obstacle::Shape::Cylinder) {
		    ROS_INFO("Obstacle [%s]: (x=%f, y=%f, r=%f)", 
		        "Cylinder",
		        obstacle.getCenter().getX(), 
		        obstacle.getCenter().getY(), 
		        obstacle.getRadius()
		    );
			currentCylinders.push_back(obstacle);
		}
	}
	
	if(currentCylinders.size() > maxCylinders.size()){
		maxCylinders.clear();
		for(auto obstacle : obstacles) 
			if(obstacle.getShape() == Obstacle::Shape::Cylinder) 
				maxCylinders.push_back(obstacle);
				
	}
}

/** 
 * @brief Callback function for the "/move_base/result" topic subscriber
 * 
 * This function is called every time a new message is published on the "/move_base/result" topic.
 * It sets the send_feedback variable to false, so that the feedback loop is stopped.
 * 
 * @param msg Result message
 */
void resultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
	send_feedback = false;
}


//DetectionAction class functions **********************************************

/**
 * @brief Constructor for the detection action server
 * 
 * @param name Name of the action server
 * @param nh ROS node handle
 */
DetectionAction::DetectionAction(std::string name, ros::NodeHandle nh) : 
    as_(nh, name, boost::bind(
            &DetectionAction::executeCB, this, _1),
            false
        )
    , action_name_(name)
    , nh_(nh)
{
    as_.start();
}

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
void DetectionAction::executeCB(const assignment_1::DetectionGoalConstPtr &goal) 
{
    // Subscribe to the "/move_base/result" topic to receive result messages
	ros::Subscriber result_subscriber = nh_.subscribe("/move_base/result", 1, resultCallback);

    // Subscribe to the "/scan_raw" topic to receive laser scan messages
	ros::Subscriber laser_sub = nh_.subscribe("scan_raw", 1000, laserCallback);
	
	// Subscribe to the "/move_base/feedback" topic to receive feedback messages
	ros::Subscriber feedback_subscriber = nh_.subscribe("/move_base/feedback", 10, positionCallback);

    maxCylinders.clear();
    ros::Rate r(5);
    
    //send position to robot
    ros::Publisher goal_publisher = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Duration(1.0).sleep();

    // Create a MoveBaseActionGoal message to store goal information
    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.goal.target_pose.header.frame_id = "map";
    goal_msg.goal.target_pose.pose.position.z = sin(goal->goal.R);
    goal_msg.goal.target_pose.pose.orientation.x = 0.0;
    goal_msg.goal.target_pose.pose.orientation.y = 0.0;
    goal_msg.goal.target_pose.pose.orientation.z = 0.0;
    goal_msg.goal.target_pose.pose.orientation.w = cos(goal->goal.R);
    goal_msg.goal.target_pose.pose.position.x = goal->goal.X;
    goal_msg.goal.target_pose.pose.position.y = goal->goal.Y;
    goal_publisher.publish(goal_msg);
    
    ROS_INFO("Goal sent to robot.");
    
    //send feedback
    send_feedback = true;
    bool success = true;
    
    //wait for robot to reach goal
    while(send_feedback){

        // get current position
        feedback_.position.X = currentPosition.getPoint().getX();
        feedback_.position.Y = currentPosition.getPoint().getY();
        feedback_.position.R = currentPosition.getOrientation();
        
        // get current cylinders
        feedback_.cylinders.current.clear();
        for(auto cyl: currentCylinders){
            assignment_1::coordinate coord;
            coord.X = cyl.getCenter().getX();
            coord.Y = cyl.getCenter().getY();
            feedback_.cylinders.current.push_back(coord);
        }
        
        // get max cylinders
        feedback_.cylinders.maximum.clear();
        for(auto cyl: maxCylinders){
            assignment_1::coordinate coord;
            coord.X = cyl.getCenter().getX();
            coord.Y = cyl.getCenter().getY();
            feedback_.cylinders.maximum.push_back(coord);
        }
        
        // check if goal is cancelled
        if (as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }
        
        // publish feedback
        as_.publishFeedback(feedback_);
        r.sleep();
    }
    
    //send result
    if(success){
      
        result_.cylinders.maximum.clear();
        result_.cylinders.current.clear();

        for(auto obstacle : maxCylinders) {
            
            assignment_1::coordinate newCoordinate;
            newCoordinate.X = obstacle.getCenter().getX();
            newCoordinate.Y = obstacle.getCenter().getY();
            
            result_.cylinders.maximum.push_back(newCoordinate);
        }

        for(auto obstacle : currentCylinders) {
            assignment_1::coordinate newCoordinate;
            newCoordinate.X = obstacle.getCenter().getX();
            newCoordinate.Y = obstacle.getCenter().getY();
            
            result_.cylinders.current.push_back(newCoordinate);
        }

        as_.setSucceeded(result_);
    }
}