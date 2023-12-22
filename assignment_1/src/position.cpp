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
 * @brief Get orientation in degrees
 * 
 * @return Orientation in degrees
 */
void Position::setPoint(CartesianPoint p){
	point = p;
}

/**
 * @brief Set orientation in radiants
 * 
 * @param o Orientation in radiants
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

void Position::setPosition(CartesianPoint p, double o){
	point = p;
	orientation = o;
}

void Position::setPosition(double x, double y, double o){
	point.setX(x);
	point.setY(y);
	orientation = o;
}

double Position::sinCosToRad(double sin, double cos){
	double angle = atan2(sin, cos);
	
	if(angle < 0) 
		angle += 2*M_PI;
	
	return angle;
}

std::ostream& operator<<(std::ostream& os, const Position& pos){
	os << "[(" << pos.getPoint().getX() << "," << pos.getPoint().getY() << "), " << pos.getOrientation() << "]";
    return os;
}
