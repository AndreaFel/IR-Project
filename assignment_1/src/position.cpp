#include "position.h"

Position::Position(){
	CartesianPoint p {};
	point = p;
	orientation = 0;
}

Position::Position(CartesianPoint p, double o) : point{p}, orientation{o}{
}

CartesianPoint Position::getPoint() const{
	return point;
}

double Position::getOrientation() const{
	return orientation;
}

void Position::setPoint(CartesianPoint p){
	point = p;
}

void Position::setPoint(double x, double y){
	point.setX(x);
	point.setY(y);
}

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
