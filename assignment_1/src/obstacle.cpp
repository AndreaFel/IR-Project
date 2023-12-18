#include "obstacle.h"
#include "ros/ros.h"

Obstacle::Obstacle(std::vector<PolarPoint> profile, 
                   Position position, 
                   const nav_msgs::OccupancyGrid::ConstPtr& map)
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

    CartesianPoint nearest_cart = nearest
                                .toCartesian()
                                .shift(position.getPoint())
                                .rotate(position.getOrientation());
    CartesianPoint first = profile.front()
                                .toCartesian()
                                .shift(position.getPoint())
                                .rotate(position.getOrientation());
    CartesianPoint last = profile.back()
                                .toCartesian()
                                .shift(position.getPoint())
                                .rotate(position.getOrientation());
    
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

    // If the radius is too small, could be a wall, check if is part of the map
    bool wall_condition = false;
    if(!radius_condition && map != nullptr)
    {
        int x = static_cast<int>(first.getX() / map->info.resolution);
        int y = static_cast<int>(first.getY() / map->info.resolution);
        int index = x + y * map->info.width;
        if (map->data[index] > 50) 
            wall_condition = true;
    }
    
    // Identify the shape
    if (circularity_condition && radius_condition && cylinder_width_condition) {
        shape_ = Shape::Cylinder;
    } else if (wall_condition) {
        shape_ = Shape::Wall;
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