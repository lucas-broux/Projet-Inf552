#include "ransac.hpp"

/**
	Constructor for the class.

	@param n_iterations The number of iterations for the algorithm.
	@param epsilon Threshold.
*/
ransac::ransac(int n_iterations, double epsilon) {
	this->n_iterations = n_iterations;
	this->epsilon = epsilon;
};

/**
	Extract the most correlated points (plane model).

	@param pointCloud The considered point cloud.
	@return List of points that correlate the most (plane model) as point3dCloud.
*/
point3dCloud ransac::fit3dPlane(point3dCloud pointCloud, bool uniformColor, Vec3b color) {
	Vec3d p1_maxRansac = pointCloud[0].getPosition();
	Vec3d p2_maxRansac = pointCloud[0].getPosition();
	Vec3d p3_maxRansac = pointCloud[0].getPosition();

	int count_maxRansac = 0;

	for (int i = 0; i < n_iterations; i++) {

		int randomIndex1 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex2 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex3 = (rand()*RAND_MAX + rand()) % pointCloud.size();

		Vec3d p1 = pointCloud[randomIndex1].getPosition();
		Vec3d p2 = pointCloud[randomIndex2].getPosition();
		Vec3d p3 = pointCloud[randomIndex3].getPosition();

		plane P = plane(p1, p2, p3);
		int count = 0;

		for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
			if (P.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
				count++;
			}
		}

		/*if (count != 0) {
			for (int index = 0; index < 10; index++) {
				cout << P.distance(pointCloud[(rand()*RAND_MAX + rand()) % pointCloud.size()].getPosition()) << " ";
			}
			cout << "/ count = " << count << endl;
		}*/

		if (count > count_maxRansac) {
			count_maxRansac = count;
			p1_maxRansac = p1;
			p2_maxRansac = p2;
			p3_maxRansac = p3;
		}
	}

	plane P = plane(p1_maxRansac, p2_maxRansac, p3_maxRansac);
	point3dCloud pointCloud_maxRansac;
	for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
		if (P.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
			if (uniformColor) {
				pointCloud_maxRansac.push_back(point3d(pointCloud[pointIndex].getPosition(), color));
			}
			else {
				pointCloud_maxRansac.push_back(pointCloud[pointIndex]);
			}
		}
	}

	return pointCloud_maxRansac;
};

/**
Extract the most correlated points (line model).

@param pointCloud The considered point cloud.
@return List of points that correlate the most (line model) as point3dCloud.
*/
point3dCloud ransac::fit3dLine(point3dCloud pointCloud, plane p, bool uniformColor, Vec3b color) {
	Vec3d p1_maxRansac = pointCloud[0].getPosition();
	Vec3d p2_maxRansac = pointCloud[0].getPosition();

	int count_maxRansac = 0;

	for (int i = 0; i < n_iterations; i++) {

		int randomIndex1 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex2 = (rand()*RAND_MAX + rand()) % pointCloud.size();

		Vec3d p1 = pointCloud[randomIndex1].getPosition();
		Vec3d p2 = pointCloud[randomIndex2].getPosition();

		line3d l = line3d(p1, p2, false);
		int count = 0;

		if (l.cosAngle(p.getDirection()) > 0.5) {
			for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
				if (l.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
					count++;
				}
			}

			/*if (count != 0) {
			for (int index = 0; index < 10; index++) {
			cout << P.distance(pointCloud[(rand()*RAND_MAX + rand()) % pointCloud.size()].getPosition()) << " ";
			}
			cout << "/ count = " << count << endl;
			}*/

			if (count > count_maxRansac) {
				count_maxRansac = count;
				p1_maxRansac = p1;
				p2_maxRansac = p2;
			}
		}
	}

	line3d l = line3d(p1_maxRansac, p2_maxRansac, false);
	point3dCloud pointCloud_maxRansac;
	
	cout << l << endl;
	cout << p << endl;
	cout << l.cosAngle(p.getDirection()) << endl;
	cout << count_maxRansac << endl;

	for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
		if (l.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
			if (uniformColor) {
				pointCloud_maxRansac.push_back(point3d(pointCloud[pointIndex].getPosition(), color));
			}
			else {
				pointCloud_maxRansac.push_back(pointCloud[pointIndex]);
			}
		}
	}
	return pointCloud_maxRansac;
};
