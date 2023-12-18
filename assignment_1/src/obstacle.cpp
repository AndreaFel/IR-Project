#include "obstacle.h"
#include "ros/ros.h"

Obstacle::Obstacle(std::vector<PolarPoint> profile)
{
    profile_ = profile;
    CartesianPoint first = profile.front().toCartesian();
    CartesianPoint last = profile.back().toCartesian();
    radius_ = CartesianPoint::distance(first, last) / 2;
    center_ = CartesianPoint::middlePoint(first, last);

    //ROS_INFO("Distance : %fcm", CartesianPoint::distance(first, last));

    double min_dist = 1000000;
    PolarPoint nearest(0,0);
    for(auto point : profile) {
        if (point.getDistance() < min_dist) {
            min_dist = point.getDistance();
            nearest = point;
        }
    }
    CartesianPoint nearest_cart = nearest.toCartesian();
    float nearest_to_first = CartesianPoint::distance(nearest_cart, first);
    float nearest_to_last = CartesianPoint::distance(nearest_cart, last);
    
    bool circularity_condition = abs(nearest_to_first - nearest_to_last) < 0.1;
    bool radius_condition = CartesianPoint::distance(nearest_cart,center_) - radius_ < 0.1;

    //ROS_INFO("radius: %f, center: (%f,%f) nearest: (%f,%f)", radius_, center_.getX(), center_.getY(), nearest_cart.getX(), nearest_cart.getY());
    //ROS_INFO("nearest_to_first: %f, nearest_to_last: %f", nearest_to_first, nearest_to_last);
    //ROS_INFO("Circularity condition: %d, radius condition: %d/n/n", circularity_condition, radius_condition);
    
    if (circularity_condition && radius_condition) {
        shape_ = Shape::Curved;
    } else {
        shape_ = Shape::Plane;
    }

    if(shape_ == Shape::Curved  && radius_ > 0.1) {
        ROS_INFO("Cylinder obstacle detected Position: (%f,%f) Radius: %f", center_.getX(), center_.getY(), radius_);
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