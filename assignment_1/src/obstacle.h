#pragma once

#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"

#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

#include "polar_point.h"
#include "cartesian_point.h"
#include "position.h"

class Obstacle {
public:
    enum class Shape { 
        Unknown,
        Cylinder, 
        Wall 
    };
    Obstacle(std::vector<PolarPoint> profile = std::vector<PolarPoint>(), 
                Position position = Position(),
                const nav_msgs::OccupancyGrid::ConstPtr& map = nullptr);

    Shape getShape() const;
    std::vector<PolarPoint> getProfile() const;
    CartesianPoint getCenter() const;
    double getRadius() const;

    static std::vector<std::vector<PolarPoint>> getObstacleProfiles(const sensor_msgs::LaserScan& m, double threshold);


private:
    std::vector<PolarPoint> profile_;
    CartesianPoint center_;
    double radius_;
    Shape shape_;
    Position position_;

};