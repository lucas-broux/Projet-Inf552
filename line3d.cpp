#include "line3d.hpp"

/**
Constructor for the class.
*/
line3d::line3d() {
	this->point = Vec3d(0, 0, 0);
	this->vector = Vec3d(0, 0, 0);
};

/**
	Constructor for the class.
	
	@param point A point in the line3d.
	@param vector A direction vector of the line3d.
*/
line3d::line3d(Vec3d point, Vec3d v, bool second_Vec3d_isvector) {
	this->point = point;
	if (second_Vec3d_isvector) {
		this->vector = v;
	}
	else {
		this->vector = v- point;
	}
};

/**
	Find the closest line3d to the given point cloud.

	@param pointcloud The considered point cloud.
	@return The line3d of linear regression.
*/
/*void line3d::regression(point3dCloud pointcloud) {
	
};*/

/**
	Find the distance between the line3d and a point.

	@param p The considered point.
*/
double line3d::distance(Vec3d p) {
	if (!this->isDegenerated()) {
		line3d AB = line3d(p, point, false);
		return(norm(product(AB.vector, vector).getVectorial())/norm(vector));
	}
	return DBL_MAX;
};

/**
Overloads ofstream.

@param os Considered stream.
@param l Considered line.
*/
ostream& operator<<(ostream& os, const line3d& l) {
	os << "Point " << l.point << endl << "Direction vector " << l.vector << endl;
	return os;
};

bool line3d::isDegenerated() {
	return(norm(vector) == 0);
};

double line3d::cosAngle(Vec3d v) {
	if (this->isDegenerated() || norm(v) == 0) {
		return 0;
	}
	return(product(vector, v).getScalar() / (norm(v)*norm(vector)));
};