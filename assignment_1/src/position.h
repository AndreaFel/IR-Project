#pragma once

#include <iostream>
#include "cartesian_point.h"
#include <cmath>

/**
 * @brief Position class
 * 
 * This class is used to represent the position of the robot in the map.
 * It stores the position of the robot in the cartesian space and its orientation.
 */

class Position {
private:
    CartesianPoint point; //position in the cartesian space
    double orientation;   //orientation in radiants

public:
    
    /**
     * @brief Constructor for the Position class
     * 
     * @param p Position in the cartesian space
     * @param o Orientation in radiants
     */
    Position(CartesianPoint p = CartesianPoint(), double o = 0);

    /**
     * @brief Get the Point object
     * 
     * @return CartesianPoint 
     */
    CartesianPoint getPoint() const;

    /**
     * @brief Get orientation in radiants
     * 
     * @return Orientation in radiants
     */
    double getOrientation() const;
	
    /**
     * @brief Set the Point object 
     * 
     * @param p Point using CartesianPoint
     */
    void setPoint(CartesianPoint p);
    /**
     * @brief Set Point object using x and y coordinates
     * 
     * @param x x coordinate
     * @param y y coordinate
     */
    void setPoint(double x, double y);
    /**
     * @brief Set orientation in radiants
     * 
     * @param o Orientation in radiants
     */
    void setOrientation(double o = 0);
	/**
     * @brief Set position in the cartesian space and orientation in radiants
     * 
     * @param p Position in the cartesian space
     * @param o Orientation in radiants
     */
    void setPosition(CartesianPoint p, double o = 0);
    /**
     * @brief Set position in the cartesian space and orientation in radiants using x and y coordinates
     * 
     * @param x x coordinate
     * @param y y coordinate
     * @param o Orientation in radiants
     */
	void setPosition(double x, double y, double o = 0);
	
    /**
     * @brief get radian angle from sin and cos
     * 
     * @param sin sin of the angle
     * @param cos cos of the angle
     * @return double  angle in radians
     */
	static double sinCosToRad(double sin, double cos);

    /**
     * @brief Shift the point by a given point
     * 
     * @param p Point to shift by
     * @return Shifted point
     */
    friend std::ostream& operator<<(std::ostream& os, const Position& pos);
};
