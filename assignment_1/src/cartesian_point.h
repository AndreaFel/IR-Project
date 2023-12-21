#pragma once

#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

class PolarPoint;

class CartesianPoint {
private:
    double x_;
    double y_;

public:
    CartesianPoint(double x = 0,double y = 0);

    double getX() const;
    double getY() const;

    void setCartesian(double x,double y);
    void setX(double x);
    void setY(double y);
    CartesianPoint& shift(CartesianPoint to_add);
    CartesianPoint& rotate(double angle_radians);

    PolarPoint to_polar();

    static CartesianPoint fromPolar(PolarPoint p);
    static CartesianPoint middlePoint(CartesianPoint a,CartesianPoint b);
    static double distance(CartesianPoint a,CartesianPoint b);

    friend std::ostream& operator<<(std::ostream& os, const CartesianPoint& point);
    friend std::ostream& operator<<(std::ostream& os, const std::vector<CartesianPoint>& points) ;


};