#include "ransac.hpp"

ransac::ransac(double probaSuccess, double epsilon) {
	this->probaSuccess = probaSuccess;
	this->epsilon = epsilon;
};

pair<point3dCloud, point3dCloud> ransac::fit3dPlane(point3dCloud pointCloud, bool uniformColor, Vec3b color) {
	//Computing the number of iteration required.
	//See https://en.wikipedia.org/wiki/Random_sample_consensus to understand the calculus.
	int n_iterations = int(log(1 - probaSuccess) / log(1 - pow(APPROX_PROPORTION_OF_POINTS_IN_THE_PLANE, 3)));
	//Adding two times the standard deviation of n_iterations to be sure of the result.
	n_iterations += int(2 * sqrt(1 - pow(APPROX_PROPORTION_OF_POINTS_IN_THE_PLANE, 3)) / pow(APPROX_PROPORTION_OF_POINTS_IN_THE_PLANE, 3));

	//Initializing the parameters of the plane.
	Vec3d p1_maxRansac = pointCloud[0].getPosition();
	Vec3d p2_maxRansac = pointCloud[0].getPosition();
	Vec3d p3_maxRansac = pointCloud[0].getPosition();
	int count_maxRansac = 0;

	//Looping over iterations.
	for (int i = 0; i < n_iterations; i++) {
		//Getting random indexes for the point cloud.
		//RAND_MAX ~ 30 000 which is inferior to the number of vertices in the pointcloud.
		int randomIndex1 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex2 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex3 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		Vec3d p1 = pointCloud[randomIndex1].getPosition();
		Vec3d p2 = pointCloud[randomIndex2].getPosition();
		Vec3d p3 = pointCloud[randomIndex3].getPosition();

		//Initializing the plane.
		plane P = plane(p1, p2, p3);
		int count = 0;

		//Calculating the number of vertices in the plane.
		for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
			if (P.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
				count++;
			}
		}

		//Updating the best plane.
		if (count > count_maxRansac) {
			count_maxRansac = count;
			p1_maxRansac = p1;
			p2_maxRansac = p2;
			p3_maxRansac = p3;
		}
	}

	//Computing the best plane.
	plane P = plane(p1_maxRansac, p2_maxRansac, p3_maxRansac);

	//Computing the point3dCloud corresponding to the best plane.
	point3dCloud pointCloud_maxRansac, pointCloud_rest;
	for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
		if (P.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
			if (uniformColor) {
				pointCloud_maxRansac.push_back(point3d(pointCloud[pointIndex].getPosition(), color, pointCloud[pointIndex].getPixelCoordinates()));
			}
			else {
				pointCloud_maxRansac.push_back(pointCloud[pointIndex]);
			}
		}
		else {
			pointCloud_rest.push_back(pointCloud[pointIndex]);
		}
	}

	return pair<point3dCloud, point3dCloud>(pointCloud_maxRansac, pointCloud_rest);
};

point3dCloud ransac::fit3dLine(point3dCloud pointCloud, plane p, bool uniformColor, Vec3b color, int nlines, double minDistBetweenLines) {
	//Computing the number of iteration required.
	//See https://en.wikipedia.org/wiki/Random_sample_consensus to understand the calculus.
	int n_iterations = int(log(1 - probaSuccess) / log(1 - pow(APPROX_PROPORTION_OF_POINTS_IN_A_LINE, 2)));
	//Adding two times the standard deviation of n_iterations to be sure of the result.
	n_iterations += int(2 * sqrt(1 - pow(APPROX_PROPORTION_OF_POINTS_IN_A_LINE, 2)) / pow(APPROX_PROPORTION_OF_POINTS_IN_A_LINE, 2));

	//Initializing the lineCloud.
	line3dCloud lineCloud;
	for (int n = 0; n < nlines; n++) {
		lineCloud.push_back(line3d(pointCloud[0].getPosition(), pointCloud[0].getPosition(), false), 0);
	}

	//Looping over iterations.
	for (int i = 0; i < n_iterations; i++) {

		//Getting random indexes for the point cloud.
		//RAND_MAX ~ 30 000 which is inferior to the number of vertices in the pointcloud.
		int randomIndex1 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		int randomIndex2 = (rand()*RAND_MAX + rand()) % pointCloud.size();
		Vec3d p1 = pointCloud[randomIndex1].getPosition();
		Vec3d p2 = pointCloud[randomIndex2].getPosition();

		//Initializing the line.
		line3d l = line3d(p1, p2, false);
		int count = 0;

		//Checking if the line is nearly orthogonal to the plane.
		if (abs(l.cosAngle(p.getDirection())) > cos(MIN_ANGLE*PI/180.)) {
			//Calculating the number of vertices in the line.
			for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
				if (l.distance(pointCloud[pointIndex].getPosition()) < epsilon) {
					count++;
				}
			}

			//Updating the best plane.
			if (count > lineCloud.getMinNpoints() && lineCloud.minDistance(l, p) > minDistBetweenLines) {
				lineCloud.set(lineCloud.getMinNpointsIndex(), l, count);
			}
		}
	}

	//Computing the point3dCloud corresponding to the best lines.
	point3dCloud pointCloud_maxRansac;
	for (int pointIndex = 0; pointIndex < pointCloud.size(); pointIndex++) {
		if (lineCloud.minDistance(pointCloud[pointIndex].getPosition()) < epsilon) {
			if (uniformColor) {
				pointCloud_maxRansac.push_back(point3d(pointCloud[pointIndex].getPosition(), color, pointCloud[pointIndex].getPixelCoordinates()));
			}
			else {
				pointCloud_maxRansac.push_back(pointCloud[pointIndex]);
			}
		}
	}
	return pointCloud_maxRansac;
};
