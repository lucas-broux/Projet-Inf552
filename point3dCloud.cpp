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
	return cloud.size();
}