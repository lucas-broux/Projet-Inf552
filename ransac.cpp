#include "ransac.hpp"

Ransac::Ransac(int n_iterations, double epsilon) {
	this->n_iterations = n_iterations;
	this->epsilon = epsilon;
};

vector<pair<Vec3d, Vec3b>> Ransac::fit(vector<pair<Vec3d, Vec3b>> pointCloud) {
	cout << "Hello world from Ransac::fit" << endl;
	for (int i = 0; i < n_iterations; i++) {

		int randomIndex1 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex2 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex3 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		cout << randomIndex1 << " " << randomIndex2 << " " << randomIndex3 << endl;

		Vec3d p1 = pointCloud[randomIndex1].first;
		Vec3d p2 = pointCloud[randomIndex2].first;
		Vec3d p3 = pointCloud[randomIndex3].first;


	}

	return pointCloud;
};