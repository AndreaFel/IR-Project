#pragma once

#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

#include "polar_point.h"
#include "cartesian_point.h"

class Obstacle {
public:
    enum class Shape { 
        Unknown,
        Curved, 
        Plane 
    };
    Obstacle(std::vector<PolarPoint> profile = std::vector<PolarPoint>());

    Shape getShape() const;
    std::vector<PolarPoint> getProfile() const;
    CartesianPoint getCenter() const;
    double getRadius() const;

private:
    std::vector<PolarPoint> profile_;
    CartesianPoint center_;
    double radius_;
    Shape shape_;

};