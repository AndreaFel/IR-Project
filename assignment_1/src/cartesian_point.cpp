#include "cartesian_point.h"
#include "polar_point.h"


CartesianPoint::CartesianPoint(double x,double y):
      x_(x)
    , y_(y)
{}

double CartesianPoint::getX() const
{
    return x_;
}
double CartesianPoint::getY() const
{
    return y_;
}

void CartesianPoint::setCartesian(double x,double y)
{
    x_ = x;
    y_ = y;
}
void CartesianPoint::setX(double x)
{
    x_ = x;
}
void CartesianPoint::setY(double y)
{
    y_ = y;
}
void CartesianPoint::shift(CartesianPoint to_add)
{
    x_ += to_add.getX();
    y_ += to_add.getY();
}

PolarPoint CartesianPoint::to_polar()
{
    return PolarPoint(sqrt(x_*x_+y_*y_),atan2(y_,x_));
}

CartesianPoint CartesianPoint::fromPolar(PolarPoint p)
{
    return CartesianPoint(p.getDistance()*cos(p.getAngleRadians()),p.getDistance()*sin(p.getAngleRadians()));
}
CartesianPoint CartesianPoint::middlePoint(CartesianPoint a,CartesianPoint b)
{
    return CartesianPoint((a.getX()+b.getX())/2,(a.getY()+b.getY())/2);
}
double CartesianPoint::distance(CartesianPoint a,CartesianPoint b)
{
    return sqrt((a.getX()-b.getX())*(a.getX()-b.getX())+(a.getY()-b.getY())*(a.getY()-b.getY()));
}

std::ostream& operator<<(std::ostream& os, const CartesianPoint& point)
{
    os << "(" << point.getX() << "," << point.getY() << ")";
    return os;
}
std::ostream& operator<<(std::ostream& os, const std::vector<CartesianPoint>& points) 
{
    os << "{";
    for (const auto& point : points) {
        os << point << " ";
    }
    os << "}";
    return os;
}


