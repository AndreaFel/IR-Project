#pragma once

#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

class CartesianPoint;

/**
 * @brief This class represents a point in polar coordinates.
 * It provides methods to convert to and from cartesian coordinates.
 * and to calculate the average and median of a vector of points.
 */
class PolarPoint {

private:
    double angle_rad;   // Angle in radians
    double distance;    // Distance from the origin

public:
    // Constructor
    /**
     * @brief Constructor for a point in polar coordinates.
     * @param ang_rad Angle in radians.
     * @param dist Distance from the origin.
     * @return A PolarPoint object.
     * @details This constructor initializes the angle and distance of the point.
     * The angle is given in radians and the distance is given in meters.
     */
    PolarPoint(double ang_rad = 0, double dist = 0);

    // Getters for polar coordinates
    /**
     * @brief Getter for the angle in radians.
     * @return The angle in radians.
     */
    double getAngleRadians() const ;
    /**
     * @brief Getter for the angle in degrees.
     * @return The angle in degrees.
     */
    double getDistance() const ;

    // Conversion functions
    /**
     * @brief Setter for the polar coordinates.
     * @param ang_rad Angle in radians.
     * @param dist Distance from the origin.
     * @details This method sets the angle and distance of the point.
     * The angle is given in radians and the distance is given in meters.
     */
    void setPolar(double ang_rad, double dist);

    /**
     * @brief Setter for the cartesian coordinates.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @details This method sets the x and y coordinates of the point.
     * The coordinates are given in meters.
     */
    void setCartesian(double x, double y) ;

    /**
     * @brief Converts the point to cartesian coordinates.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @details This method converts the point to cartesian coordinates.
     * The coordinates are given in meters.
     */
    void convertToCartesian(double &x, double &y) const;

    CartesianPoint toCartesian() const;

    // Static methods
    /**
     * @brief Calculates the average point of a vector of points.
     * @param points Vector of points.
     * @return The average point.
     * @details This method calculates the average point of a vector of points.
     * The average point is calculated as the average of the angles and the
     * average of the distances of the points in the vector.
     */
    static PolarPoint getAveragePoint(const std::vector<PolarPoint>& points);

    /**
     * @brief Calculates the median point of a vector of points.
     * @param points Vector of points.
     * @return The median point.
     * @details This method calculates the median point of a vector of points.
     * The median point is calculated as the median of the angles and the
     * median of the distances of the points in the vector.
     */
    static PolarPoint getMedianPoint(const std::vector<PolarPoint>& points) ;

    static double distanceBetweenPoints(const PolarPoint& point1, const PolarPoint& point2);

    static std::pair<PolarPoint, PolarPoint> getClosestPoints(std::vector<PolarPoint>& points);

    static std::pair<double,double> getMiddlePoint(const PolarPoint& point1, const PolarPoint& point2);
    
    // Output stream operators overload
    /**
     * @brief Output stream operator for a PolarPoint.
     * @param os Output stream.
     * @param point Point to be printed.
     * @return The output stream.
     * @details This method overloads the output stream operator for a PolarPoint.
     * It prints the point in the format [angle,distance].
     */
    friend std::ostream& operator<<(std::ostream& os, const PolarPoint& point) ;

    /**
     * @brief Output stream operator for a vector of PolarPoints.
     * @param os Output stream.
     * @param points Vector of points to be printed.
     * @return The output stream.
     * @details This method overloads the output stream operator for a vector of PolarPoints.
     * It prints the vector in the format {point1,point2,...}.
     */
    friend std::ostream& operator<<(std::ostream& os, const std::vector<PolarPoint>& points) ;
};
