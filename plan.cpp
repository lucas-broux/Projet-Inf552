#include "plan.hpp"

/**
Constructor for the class.

*/
Plan::Plan() {
	this->a = 0.;
	this->b = 0.;
	this->c = 0.;
	this->d = 0.;
};

/**
	Constructor for the class.
	
	@param a 1st parameter of a*x + b*y + c*z + d = 0
	@param b 2nd parameter of a*x + b*y + c*z + d = 0
	@param c 3rd parameter of a*x + b*y + c*z + d = 0
	@param d 4th parameter of a*x + b*y + c*z + d = 0
*/
Plan::Plan(double a, double b, double c, double d) {
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
};

/**
	Constructor for the class.
	
	@param p1 1st point of the plan
	@param p2 2nd point of the plan
	@param p3 3rd point of the plan
*/
Plan::Plan(Vec3d p1, Vec3d p2, Vec3d p3) {
	/* 
	We define
		X = [[1; x1; y1]
			 [1; x2; y2]
			 [1; x3; y3]]
		y = [z1, z2, z3]
	If  X.t() * X is not inversible, degenerate case.
	Else [d, a, b] = X.inv() * y and c = -1.
	*/
	// Compute X and y.
	Mat X(3, 3, CV_64FC1);
	X.at<double>(0, 0) = 1;
	X.at<double>(0, 1) = p1[0];
	X.at<double>(0, 2) = p1[1];
	X.at<double>(1, 0) = 1;
	X.at<double>(1, 1) = p2[0];
	X.at<double>(1, 2) = p2[1];
	X.at<double>(2, 0) = 1;
	X.at<double>(2, 1) = p3[0];
	X.at<double>(2, 2) = p3[1];
	
	Mat y(3, 1, CV_64FC1);
	y.at<double>(0, 0) = p1[2];
	y.at<double>(1, 0) = p2[2];
	y.at<double>(2, 0) = p3[2];
	
	if (determinant(X) < 0.001){
		// Degenerate case.
		this->a = 0.;
		this->b = 0.;
		this->c = 0.;
		this->d = 0.;
	}
	else {
		Mat alpha = X.inv() * y;
		this->a = alpha.at<double>(1, 0);
		this->b = alpha.at<double>(2, 0);
		this->c = -1.0;
		this->d = alpha.at<double>(0, 0);
	}
};

/**
	Find the closest plan to the given point cloud.

	@param pointcloud The considered point cloud.
	@return The Plan of linear regression.
*/
void Plan::regression(point3dCloud pointcloud) {
	/*
	We define
		X = [[1; x1; y1]
			 [1; x2; y2]
			 ...
			 [1; xn; yn]]
	y = [z1, zn, zn]
	If  X.t() * X is not inversible, degenerate case.
	Else [d, a, b] = (((X.t() * X).inv()) * (X.t())) * y and c = -1;
	*/
	// Compute X and y.
	Mat X(pointcloud.size(), 3, CV_64FC1);
	Mat y(pointcloud.size(), 1, CV_64FC1);
	for (int point_counter = 0; point_counter < pointcloud.size(); point_counter++) {
		Vec3d point = pointcloud[point_counter].getPosition();
		X.at<double>(point_counter, 0) = 1.;
		X.at<double>(point_counter, 1) = point[0];
		X.at<double>(point_counter, 2) = point[1];
		y.at<double>(point_counter, 0) = point[2];
	}

	// Compute coefficients.
	if (determinant((X.t() * X)) < 0.001) {
		a = 0.;
		b = 0.;
		c = 0.;
		d = 0.;
	}
	else {
		Mat alpha = (((X.t() * X).inv()) * (X.t())) * y;
		a = alpha.at<double>(1, 0);
		b = alpha.at<double>(2, 0);
		c = -1.0;
		d = alpha.at<double>(0, 0);
	}
}


/**
	Find the distance between the plan and a point.

	@param p The considered point.
*/
double Plan::distance(Vec3d p) {
	if (!this->isDegenerated() && sqrt(a*a + b*b + c*c) != 0) {
		return(abs(a*p[0] + b*p[1] + c*p[2] + d) / sqrt(a*a + b*b + c*c));
	}
	return DBL_MAX;
}

/**
Overloads ofstream.

@param os Considered stream.
@param p Considered plan.
*/
ostream& operator<<(ostream& os, const Plan& p) {
	if (p.a < 0) {
		os << "- " << -p.a << " * x ";
	}
	else {
		if (p.a != 0) {
			os << p.a << " * x ";
		}
	}
	if (p.b < 0) {
		os << "- " << -p.b << " * y ";
	}
	else {
		if (p.b != 0) {
			os << "+ " << p.b << " * y ";
		}
	}
	if (p.c < 0) {
		os << "- " << -p.c << " * z ";
	}
	else {
		if (p.c != 0) {
			os << "+ " << p.c << " * z ";
		}
	}
	if (p.d < 0) {
		os << "- " << -p.d << " = 0";
	}
	else {
		if (p.d != 0) {
			os << "+ " << p.d << " = 0";
		}
	}
	return os;
};

bool Plan::isDegenerated() {
	return(a == 0 && b == 0 && c == 0 && d == 0);
};