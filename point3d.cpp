#include "point3d.hpp"

/**
Constructor for the class.

@param position The position of the 3dPoint.
@param color The color of the 3dPoint.
*/
point3d::point3d(Vec3d position, Vec3b color) {
	this->position = position;
	this->color = color;
};

Vec3d point3d::getPosition() {
	return position;
};

Vec3b point3d::getColor() {
	return color;
};

