#include "plan.hpp"

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
	this->a = 0;
	this->b = 0;
	this->c = 0;
	this->d = 0;
};