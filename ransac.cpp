#include "ransac.hpp"

/**
	Constructor for the class.

	@param n_iterations The number of iterations for the algorithm.
	@param epsilon Threshold.
*/
Ransac::Ransac(int n_iterations, double epsilon) {
	this->n_iterations = n_iterations;
	this->epsilon = epsilon;
};


/**
	Extract the most correlated points (plane model).

	@param pointCloud The considered point cloud.
	@return List of points that correlate the most (plane model) as vector<pair<Vec3d, Vec3b>>.
*/
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

		Plan P = Plan(p1, p2, p3);
		int count = 0;

		for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
			if (P.distance(pointCloud[pointIndex].first) < epsilon) {
				count++;
			}
		}
	}

	return pointCloud;
};