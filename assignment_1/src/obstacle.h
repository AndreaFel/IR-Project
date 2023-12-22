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

/**
 * @brief Obstacle class
 * 
 * This class is used to represent an obstacle in the environment.
 * It stores the obstacle profile, the center and the radius of the obstacle.
 * It also stores the position of the obstacle in the map.
 */
class Obstacle {
public:
    // enum class for the shape of the obstacle
    enum class Shape { 
        Unknown,
        Cylinder
    };

    /**
     * @brief Constructor for the Obstacle class
     * 
     * This constructor is used to create an obstacle with a given profile and position.
     * The shape of the obstacle is identified using a chain of rules on the obstacle profile.
     * 
     * @param profile Obstacle profile
     * @param position Position of the obstacle in the map
     */
    Obstacle(std::vector<PolarPoint> profile = std::vector<PolarPoint>(), 
                Position position = Position());

    /**
     * @brief Get the Shape object
     * 
     * @return Shape enum class
     */
    Shape getShape() const;

    /**
     * @brief Get the Profile object
     * 
     * @return Profile of the obstacle  (vector of polar points)
     */
    std::vector<PolarPoint> getProfile() const;

    /**
     * @brief Get the Center of the obstacle
     * 
     * @return Center of the obstacle (cartesian point)
     */
    CartesianPoint getCenter() const;

    /**
     * @brief Get the radius of the obstacle
     * 
     * @return Radius of the obstacle in meters
     */
    double getRadius() const;

    /**
     * @brief Get the Obstacle Profiles object
     * 
     * This function is used to extract the obstacle profiles from the laser scan message. Given a threshold, 
     * it identifies the obstacle profiles in the laser scan message. If the signal make a jump of more than
     * the threshold, it is considered a new obstacle profile.
     * 
     * @param m Laser scan message
     * @param threshold Threshold to identify the obstacle profiles
     * @return Vector of obstacle profiles
     */
    static std::vector<std::vector<PolarPoint>> getObstacleProfiles(const sensor_msgs::LaserScan& m, double threshold);


private:
    std::vector<PolarPoint> profile_; // Obstacle profile
    CartesianPoint center_;           // Center of the obstacle
    double radius_;                   // Radius of the obstacle
    Shape shape_;                     // Shape of the obstacle                 
    Position position_;               // Position of the obstacle in the map

};
