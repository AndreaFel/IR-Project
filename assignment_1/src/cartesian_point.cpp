#include "cartesian_point.h"
#include "polar_point.h"

/**
 * @brief Constructor for the CartesianPoint class
 * 
 * @param x x coordinate
 * @param y y coordinate
 */
CartesianPoint::CartesianPoint(double x,double y):
      x_(x)
    , y_(y)
{}

/**
 * @brief Get the x coordinate
 * 
 * @return x coordinate
 */
double CartesianPoint::getX() const
{
    return x_;
}
/**
 * @brief Get the y coordinate
 * 
 * @return y coordinate
 */
double CartesianPoint::getY() const
{
    return y_;
}

/**
 * @brief Set the x and y coordinates
 * 
 * @param x x coordinate
 * @param y y coordinate
 */
void CartesianPoint::setCartesian(double x,double y)
{
    x_ = x;
    y_ = y;
}

/**
 * @brief Set the x coordinate
 * 
 * @param x x coordinate
 */
void CartesianPoint::setX(double x)
{
    x_ = x;
}

/**
 * @brief Set the y coordinate
 * 
 * @param y y coordinate
 */
void CartesianPoint::setY(double y)
{
    y_ = y;
}

/**
 * @brief Shift the point by a given point
 * 
 * @param to_add point to add
 * @return reference to the shifted point
 */
CartesianPoint& CartesianPoint::shift(CartesianPoint to_add)
{
    x_ += to_add.getX();
    y_ += to_add.getY();
    return *this;
}

/**
 * @brief Rotate the point by a given angle
 * 
 * @param angle_radians angle in radians
 * @return reference to the rotated point
 */
CartesianPoint& CartesianPoint::rotate(double angle_radians)
{
    double x = x_;
    double y = y_;
    x_ = x*cos(angle_radians)-y*sin(angle_radians);
    y_ = x*sin(angle_radians)+y*cos(angle_radians);
    return *this;
}

/**
 * @brief Get the polar coordinates of the point
 * 
 * @return PolarPoint object
 */
PolarPoint CartesianPoint::to_polar()
{
    return PolarPoint(sqrt(x_*x_+y_*y_),atan2(y_,x_));
}

/**
 * @brief Get the polar coordinates of the point
 * 
 * @return PolarPoint object
 */
CartesianPoint CartesianPoint::fromPolar(PolarPoint p)
{
    return CartesianPoint(p.getDistance()*cos(p.getAngleRadians()),p.getDistance()*sin(p.getAngleRadians()));
}

/**
 * @brief Get the middle point between two points
 * 
 * @param a first point
 * @param b second point
 * @return middle point
 */
CartesianPoint CartesianPoint::middlePoint(CartesianPoint a,CartesianPoint b)
{
    return CartesianPoint((a.getX()+b.getX())/2,(a.getY()+b.getY())/2);
}

/**
 * @brief Get the distance between two points
 * 
 * @param a first point
 * @param b second point
 * @return distance between the two points
 */
double CartesianPoint::distance(CartesianPoint a,CartesianPoint b)
{
    return sqrt((a.getX()-b.getX())*(a.getX()-b.getX())+(a.getY()-b.getY())*(a.getY()-b.getY()));
}

/**
 * @brief Overload of the << operator for the CartesianPoint class
 * 
 * @param os output stream
 * @param point CartesianPoint object
 * @return output stream
 */
std::ostream& operator<<(std::ostream& os, const CartesianPoint& point)
{
    os << "(" << point.getX() << "," << point.getY() << ")";
    return os;
}
/**
 * @brief Overload of the << operator for std::vector<CartesianPoint> objects
 * 
 * @param os output stream
 * @param points vector of CartesianPoint objects
 * @return output stream
 */
std::ostream& operator<<(std::ostream& os, const std::vector<CartesianPoint>& points) 
{
    os << "{";
    for (const auto& point : points) {
        os << point << " ";
    }
    os << "}";
    return os;
}


