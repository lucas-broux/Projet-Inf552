#include "point3dCloud.hpp"

/**
Constructor for the class.
*/
point3dCloud::point3dCloud() {
	
};

void point3dCloud::push_back(point3d point) {
	cloud.push_back(point);
};

point3d point3dCloud::operator[](int i) {
	return(cloud[i]);
};

int point3dCloud::size() {
	return (int)cloud.size();
}

double point3dCloud::meanNeighboursDistance() {
	double d = 0;

	if (this->size() < 1000) {
		for (int i = 0; i < this->size(); i++) {
			double temp_d = DBL_MAX;
			for (int j = 0; j < this->size(); j++) {
				if (i != j && cloud[i].distance(cloud[j]) < temp_d) {
					temp_d = cloud[i].distance(cloud[j]);
				}
			}
			d += temp_d;
		}
		return(d / this->size());
	}
	else {
		for (int i = 0; i < 1000; i++) {
			int randomIndex1 = (rand()*RAND_MAX + rand()) % this->size();
			double temp_d = DBL_MAX;
			for (int j = 0; j < 10000; j++) {
				int randomIndex2 = (rand()*RAND_MAX + rand()) % this->size();
				if (cloud[randomIndex1].distance(cloud[randomIndex2]) < temp_d) {
					temp_d = cloud[randomIndex1].distance(cloud[randomIndex2]);
				}
			}
			d += temp_d;
		}
		return(d / 1000);
	}
}