#include "point3d.hpp"

/**
Constructor for the class.

@param position The position of the 3dPoint.
@param color The color of the 3dPoint.
*/
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

