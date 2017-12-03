#include "product.hpp"

product::product(Vec3d v1, Vec3d v2) {
	this->v1 = v1;
	this->v2 = v2;
};

double product::getScalar() {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
};

Vec3d product::getVectorial() {
	Vec3d v;
	v[0] = v1[1] * v2[2] - v1[2] * v2[1];
	v[1] = v1[2] * v2[0] - v1[0] * v2[2];
	v[2] = v1[0] * v2[1] - v1[1] * v2[0];
	return v;
};