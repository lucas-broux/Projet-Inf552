#include "point3d.hpp"

point3d::point3d(Vec3d position, Vec3b color, pair<int, int> pixel) {
	this->position = position;
	this->color = color;
	this->pixel = pixel;
};

Vec3d point3d::getPosition() {
	return position;
};

Vec3b point3d::getColor() {
	return color;
};

pair<int, int> point3d::getPixelCoordinates() {
	return pixel;
};

double point3d::distance(point3d p) {
	return(sqrt((position[0] - p.position[0])*(position[0] - p.position[0]) +
				(position[1] - p.position[1])*(position[1] - p.position[1]) +
				(position[2] - p.position[2])*(position[2] - p.position[2]) ));
};

double point3d::distance(Vec3d v) {
	return(sqrt((position[0] - v[0])*(position[0] - v[0]) +
		(position[1] - v[1])*(position[1] - v[1]) +
		(position[2] - v[2])*(position[2] - v[2])));
};

bool  point3d::operator==(point3d p) {
	return(this->getPosition() == p.getPosition());
};

void point3d::setColor(Vec3b color) {
	this->color = color;
};

void point3d::setPosition(Vec3d position) {
	this->position = position;
};

