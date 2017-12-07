#include "color.hpp"

vector<Vec3b> color::defineColors() {
	vector<Vec3b> color;
	color.push_back(Vec3b(0, 0, 255));
	color.push_back(Vec3b(0, 255, 0));
	color.push_back(Vec3b(255, 0, 0));
	color.push_back(Vec3b(0, 255, 255));
	color.push_back(Vec3b(255, 0, 255));
	color.push_back(Vec3b(255, 255, 0));

	color.push_back(Vec3b(128, 0, 255));
	color.push_back(Vec3b(128, 255, 0));
	color.push_back(Vec3b(255, 128, 0));
	color.push_back(Vec3b(0, 128, 255));
	color.push_back(Vec3b(0, 255, 128));
	color.push_back(Vec3b(255, 0, 128));
	color.push_back(Vec3b(0, 128, 128));
	color.push_back(Vec3b(128, 0, 128));
	color.push_back(Vec3b(128, 128, 0));

	return color;
};