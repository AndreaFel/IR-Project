#include "position.h"

/**
 * @brief Constructor for the Position class
 * 
 * @param p Position in the cartesian space
 * @param o Orientation in radiants
 */
Position::Position(CartesianPoint p, double o) : point{p}, orientation{o}{
}

/**
 * @brief Get the Point object
 * 
 * @return CartesianPoint 
 */
CartesianPoint Position::getPoint() const{
	return point;
}

/**
 * @brief Get orientation in radiants
 * 
 * @return Orientation in radiants
 */
double Position::getOrientation() const{
	return orientation;
}

/**
 * @brief Set the Point object 
 * 
 * @param p Point using CartesianPoint
 */
void Position::setPoint(CartesianPoint p){
	point = p;
}

/**
 * @brief Set Point object using x and y coordinates
 * 
 * @param x x coordinate
 * @param y y coordinate
 */
void Position::setPoint(double x, double y){
	point.setX(x);
	point.setY(y);
}

/**
 * @brief Set orientation in degrees
 * 
 * @param o Orientation in degrees
 */
void Position::setOrientation(double o){
	orientation = o;
}

/**
 * @brief Set position in the cartesian space and orientation in radiants
 * 
 * @param p Position in the cartesian space
 * @param o Orientation in radiants
 */
void Position::setPosition(CartesianPoint p, double o){
	point = p;
	orientation = o;
}

/**
 * @brief Set position in the cartesian space and orientation in radiants using x and y coordinates
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param o Orientation in radiants
 */
void Position::setPosition(double x, double y, double o){
	point.setX(x);
	point.setY(y);
	orientation = o;
}

/**
 * @brief get radian angle from sin and cos
 * 
 * @param sin sin of the angle
 * @param cos cos of the angle
 * @return double  angle in radians
 */
double Position::sinCosToRad(double sin, double cos){
	double angle = atan2(sin, cos);
	
	if(angle < 0) 
		angle += 2*M_PI;
	
	return angle;
}

/**
 * @brief Shift the point by a given point
 * 
 * @param p Point to shift by
 * @return Shifted point
 */
std::ostream& operator<<(std::ostream& os, const Position& pos){
	os << "[(" << pos.getPoint().getX() << "," << pos.getPoint().getY() << "), " << pos.getOrientation() << "]";
    return os;
}
