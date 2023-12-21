#include "obstacle.h"
#include "ros/ros.h"
#include <iostream>

Obstacle::Obstacle(std::vector<PolarPoint> profile, 
                   Position position)
{
    // Store data point and observer position as reference
    profile_ = profile;
    position_ = position;

    // extract the nearest relative point from the profile
    double min_dist = 1000000;
    PolarPoint nearest(0,0);

    for(auto point : profile) {
        if (point.getDistance() < min_dist) {
            min_dist = point.getDistance();
            nearest = point;
        }
    }

    // Get the three cartesian points, the nearest point, the first and the last
    // each point is shifted to the robot position and rotated to the robot orientation

    CartesianPoint nearest_cart = nearest.toCartesian();
    CartesianPoint first = profile.front().toCartesian();
    CartesianPoint last = profile.back().toCartesian();
    
    // Compute obstacle center and radius from the three points
    radius_ = CartesianPoint::distance(first, last) / 2;
    center_ = CartesianPoint::middlePoint(first, last);

    // Compute the circularity condition
    float nearest_to_first = CartesianPoint::distance(nearest_cart, first);
    float nearest_to_last = CartesianPoint::distance(nearest_cart, last);
    
    // Check the condition for shape identification
    bool circularity_condition = abs(nearest_to_first - nearest_to_last) < 0.1;
    bool radius_condition = CartesianPoint::distance(nearest_cart,center_) - radius_ < 0.1;

    // The Cylinder to detect is ~17cm radius
    bool cylinder_width_condition = radius_ > 0.1 && radius_ < 0.2;
	
	// Anti-false positive check: every point of the cylinder profile should be at the same distance from the center
	bool constant_radius_condition = true;
	if (circularity_condition && radius_condition && cylinder_width_condition) {
		int i = 0;
		for(PolarPoint p : profile){
			CartesianPoint cp = p.toCartesian();
			double dist = CartesianPoint::distance(center_, cp);
    		if(abs(dist - radius_) > 0.1){
    			std::cout<<cp.getX()<<","<<cp.getY()<<"; "<<center_.getX()<<","<<center_.getY()<<"; "<<radius_<<"; "<<dist<<"; "<<i<<"; "<<profile.size()<<std::endl;
    			//std::cin>>dist;
    			constant_radius_condition = false;
    			break;
    		}
    		i++;
		}
	}
	
	center_ = center_.shift(position.getPoint()).rotate(position.getOrientation());
    
    // Identify the shape
    if (circularity_condition 
    	&& radius_condition 
    	&& cylinder_width_condition 
    	&& constant_radius_condition) {
        
        shape_ = Shape::Cylinder;
    
    } else {
        shape_ = Shape::Unknown;
    }

}

Obstacle::Shape Obstacle::getShape() const
{
    return shape_;
}
std::vector<PolarPoint> Obstacle::getProfile() const
{
    return profile_;
}
CartesianPoint Obstacle::getCenter() const
{
    return center_;
}
double Obstacle::getRadius() const
{
    return radius_;
}

std::vector<std::vector<PolarPoint>> Obstacle::getObstacleProfiles(const sensor_msgs::LaserScan& m, double threshold)
{
    std::vector<std::vector<PolarPoint>> profiles;
    std::vector<PolarPoint> profile;
    int index = 0;    
    double prev = m.ranges.front();

    for(auto dist : m.ranges) {
        if (abs(dist - prev) > threshold) {
            profiles.push_back(profile);
            profile.clear();
        }
            
        profile.push_back(PolarPoint( m.angle_min + (index++ * m.angle_increment),dist));
        prev = dist;
    }
    return profiles;
}
