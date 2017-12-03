#include "line3d.hpp"

line3d::line3d() {
	this->point = Vec3d(0, 0, 0);
	this->vector = Vec3d(0, 0, 0);
};

line3d::line3d(Vec3d point, Vec3d v, bool second_Vec3d_isvector) {
	this->point = point;
	if (second_Vec3d_isvector) {
		this->vector = v;
	}
	else {
		this->vector = v- point;
	}
};

Vec3d line3d::getVector() {
	return vector;
}

Vec3d line3d::getPoint() {
	return point;
}

double line3d::distance(Vec3d p) {
	if (!this->isDegenerated()) {
		line3d AB = line3d(p, point, false);
		return(norm(product(AB.vector, vector).getVectorial())/norm(vector));
	}
	return DBL_MAX;
};

double line3d::distance(line3d l) {
	if (!this->isDegenerated() && !l.isDegenerated()) {
		Vec3d AB = l.point - point;
		Mat X(3, 3, CV_64FC1);
		X.at<double>(0, 0) = AB[0];
		X.at<double>(0, 1) = AB[1];
		X.at<double>(0, 2) = AB[2];
		X.at<double>(1, 0) = l.vector[0];
		X.at<double>(1, 1) = l.vector[1];
		X.at<double>(1, 2) = l.vector[2];
		X.at<double>(2, 0) = vector[0];
		X.at<double>(2, 1) = vector[1];
		X.at<double>(2, 2) = vector[2];
		return(abs(determinant(X)) / norm(product(vector, l.vector).getVectorial()));
	}
	return DBL_MAX;
};

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