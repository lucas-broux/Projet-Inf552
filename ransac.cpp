#include "ransac.hpp"

Ransac::Ransac(int n_iterations, double epsilon) {
	this->n_iterations = n_iterations;
	this->epsilon = epsilon;
};

vector<pair<Vec3d, Vec3b>> Ransac::fit(vector<pair<Vec3d, Vec3b>> pointCloud) {
	for (int i = 0; i < n_iterations; i++) {
		//ransac algo
	}

	return pointCloud;
};