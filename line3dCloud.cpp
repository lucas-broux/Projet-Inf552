#include "line3dCloud.hpp"

line3dCloud::line3dCloud() {
	
};

void line3dCloud::push_back(line3d line, int npoints) {
	cloud.push_back(line);
	this->npoints.push_back(npoints);
};

line3d line3dCloud::operator[](int i) {
	return(cloud[i]);
};

int line3dCloud::size() {
	return (int)cloud.size();
};

int line3dCloud::getMinNpointsIndex() {
	int minIndex = 0;
	for (int i = 0; i < this->size(); i++) {
		if (npoints[i] < npoints[minIndex]) {
			minIndex = i;
		}
	}
	return minIndex;
};

int line3dCloud::getMinNpoints() {
	int minNpoints = 0;
	for (int i = 0; i < this->size(); i++) {
		if (npoints[i] < minNpoints) {
			minNpoints = npoints[i];
		}
	}
	return minNpoints;
};

void line3dCloud::set(int i, line3d line, int npoints) {
	cloud[i] = line;
	this->npoints[i] = npoints;
}

double line3dCloud::minDistance(line3d line) {
	double minDistance = DBL_MAX;
	for (int i = 0; i < this->size(); i++) {
		if (cloud[i].distance(line) < minDistance) {
			minDistance = cloud[i].distance(line);
		}
	}
	return minDistance;
};

double line3dCloud::minDistance(Vec3d v) {
	double minDistance = DBL_MAX;
	for (int i = 0; i < this->size(); i++) {
		if (cloud[i].distance(v) < minDistance) {
			minDistance = cloud[i].distance(v);
		}
	}
	return minDistance;
};