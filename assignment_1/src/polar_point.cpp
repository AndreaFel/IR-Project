#include "polar_point.h"

// Constructor
/**
 * @brief Constructor for a point in polar coordinates.
 * @param ang_rad Angle in radians.
 * @param dist Distance from the origin.
 * @return A PolarPoint object.
 * @details This constructor initializes the angle and distance of the point.
 * The angle is given in radians and the distance is given in meters.
 */
PolarPoint::PolarPoint(double ang_rad, double dist) : angle_rad(ang_rad), distance(dist) {}

// Getters for polar coordinates
/**
 * @brief Getter for the angle in radians.
 * @return The angle in radians.
 */
double PolarPoint::getAngleRadians() const {
    return angle_rad;
}

/**
 * @brief Getter for the angle in degrees.
 * @return The angle in degrees.
 */
double PolarPoint::getDistance() const {
    return distance;
}

// Conversion functions
/**
 * @brief Setter for the polar coordinates.
 * @param ang_rad Angle in radians.
 * @param dist Distance from the origin.
 * @details This method sets the angle and distance of the point.
 * The angle is given in radians and the distance is given in meters.
 */
void PolarPoint::setPolar(double ang_rad, double dist) {
    angle_rad = ang_rad;
    distance = dist;
}

/**
 * @brief Setter for the cartesian coordinates.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @details This method sets the x and y coordinates of the point.
 * The coordinates are given in meters.
 */
void PolarPoint::setCartesian(double x, double y) {
    distance = sqrt(x * x + y * y);
    angle_rad = atan2(y, x);
}

/**
 * @brief Converts the point to cartesian coordinates.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @details This method converts the point to cartesian coordinates.
 * The coordinates are given in meters.
 */
void PolarPoint::convertToCartesian(double &x, double &y) const {
    x = distance * cos(angle_rad);
    y = distance * sin(angle_rad);
}

// Static methods
/**
 * @brief Calculates the average point of a vector of points.
 * @param points Vector of points.
 * @return The average point.
 * @details This method calculates the average point of a vector of points.
 * The average point is calculated as the average of the angles and the
 * average of the distances of the points in the vector.
 */
PolarPoint PolarPoint::getAveragePoint(const std::vector<PolarPoint>& points) {
    double total_distance = .0;
    double total_angle = .0;

    for (const auto& point : points) {
        total_distance += point.getDistance();
        total_angle += point.getAngleRadians();
    }

    return PolarPoint(total_angle / points.size(), total_distance / points.size());
}

/**
 * @brief Calculates the median point of a vector of points.
 * @param points Vector of points.
 * @return The median point.
 * @details This method calculates the median point of a vector of points.
 * The median point is calculated as the median of the angles and the
 * median of the distances of the points in the vector.
 */
PolarPoint PolarPoint::getMedianPoint(const std::vector<PolarPoint>& points) {
	std::vector<PolarPoint> pointsCopy {points};
	
    std::sort(pointsCopy.begin(), pointsCopy.end(),
                [](const PolarPoint& a, const PolarPoint& b) {
                    return a.getDistance() < b.getDistance();
                });

    return pointsCopy[pointsCopy.size() / 2];
}

double PolarPoint::distanceBetweenPoints(const PolarPoint& point1, const PolarPoint& point2)
{
    double x1, y1, x2, y2;
    point1.convertToCartesian(x1, y1);
    point2.convertToCartesian(x2, y2);
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

std::pair<PolarPoint, PolarPoint> PolarPoint::getClosestPoints(std::vector<PolarPoint>& points)
{
    if (points.size() < 2) {
        std::cerr << "Insufficient points to form pairs." << std::endl;
        return std::make_pair(points[0], points[0]);
    }

    double minDistance = std::numeric_limits<double>::max();
    std::pair<PolarPoint, PolarPoint> closestPair = std::make_pair(points[0], points[1]);

    int to_remove_first, to_remove_second;

    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            double currentDistance = distanceBetweenPoints(points[i], points[j]);
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closestPair = std::make_pair(points[i], points[j]);
                to_remove_first = i;
                to_remove_second = j;
            }
        }
    }

    points.erase(points.begin() + to_remove_first);
    points.erase(points.begin() + to_remove_second - 1);

    return closestPair;

}

std::pair<double,double> PolarPoint::getMiddlePoint(const PolarPoint& point1, const PolarPoint& point2)
{
    double x1, y1, x2, y2;
    point1.convertToCartesian(x1, y1);
    point2.convertToCartesian(x2, y2);
    return std::make_pair((x1 + x2) / 2, (y1 + y2) / 2);
}

// Output stream operators overload
/**
 * @brief Output stream operator for a PolarPoint.
 * @param os Output stream.
 * @param point Point to be printed.
 * @return The output stream.
 * @details This method overloads the output stream operator for a PolarPoint.
 * It prints the point in the format [angle,distance].
 */
std::ostream& operator<<(std::ostream& os, const PolarPoint& point) {
    os << "[" << point.angle_rad << "," << point.distance<<"]";
    return os;
}

/**
 * @brief Output stream operator for a vector of PolarPoints.
 * @param os Output stream.
 * @param points Vector of points to be printed.
 * @return The output stream.
 * @details This method overloads the output stream operator for a vector of PolarPoints.
 * It prints the vector in the format {point1,point2,...}.
 */
std::ostream& operator<<(std::ostream& os, const std::vector<PolarPoint>& points) {
    os << "{";
    for (const auto& point : points) {
        os << point;
    }
    os << "}";
    return os;
}
