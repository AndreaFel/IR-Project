#pragma once

#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

class PolarPoint;

/**
 * @brief CartesianPoint class
 * 
 * This class is used to represent a point in the cartesian space.
 * It stores the x and y coordinates of the point.
 */
class CartesianPoint {
private:
    double x_;  // x coordinate
    double y_;  // y coordinate

public:
    /**
     * @brief Constructor for the CartesianPoint class
     * 
     * @param x x coordinate
     * @param y y coordinate
     */
    CartesianPoint(double x = 0,double y = 0);

    /**
     * @brief Get the x coordinate
     * 
     * @return x coordinate
     */
    double getX() const;

    /**
     * @brief Get the y coordinate
     * 
     * @return y coordinate
     */
    double getY() const;

    /**
     * @brief Set the x and y coordinates
     * 
     * @param x x coordinate
     * @param y y coordinate
     */
    void setCartesian(double x,double y);
    /**
     * @brief Set the x coordinate
     * 
     * @param x x coordinate
     */
    void setX(double x);
    /**
     * @brief Set the y coordinate
     * 
     * @param y y coordinate
     */
    void setY(double y);

    /**
     * @brief Shift the point by a given point
     * 
     * @param to_add point to add
     * @return reference to the shifted point
     */
    CartesianPoint& shift(CartesianPoint to_add);
    /**
     * @brief Rotate the point by a given angle
     * 
     * @param angle_radians angle in radians
     * @return reference to the rotated point
     */
    CartesianPoint& rotate(double angle_radians);

    /**
     * @brief Convert the point to polar coordinates
     * 
     * @return PolarPoint object
     */
    PolarPoint to_polar();

    /**
     * @brief Convert a polar point to a cartesian point
     * 
     * @param p PolarPoint object
     * @return CartesianPoint object
     */
    static CartesianPoint fromPolar(PolarPoint p);
    /**
     * @brief Get the middle point between two points
     * 
     * @param a first point
     * @param b second point
     * @return middle point
     */
    static CartesianPoint middlePoint(CartesianPoint a,CartesianPoint b);
    /**
     * @brief Get the distance between two points
     * 
     * @param a first point
     * @param b second point
     * @return distance between the two points
     */
    static double distance(CartesianPoint a,CartesianPoint b);

    /**
     * @brief Overload of the << operator for CartesianPoint objects
     * 
     * @param os output stream
     * @param point CartesianPoint object
     * @return output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const CartesianPoint& point);
    /**
     * @brief Overload of the << operator for std::vector<CartesianPoint> objects
     * 
     * @param os output stream
     * @param points vector of CartesianPoint objects
     * @return output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const std::vector<CartesianPoint>& points) ;


};