#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

//#include "point3dCloud.hpp"
#include "product.hpp"

using namespace std;
using namespace cv;

class line3d {
private:
	Vec3d point;
	Vec3d vector;

public:

	line3d();

	line3d(Vec3d point, Vec3d v, bool second_Vec3d_isvector);

	double distance(Vec3d p);

	double distance(line3d l);

	//void regression(point3dCloud pointcloud);

	friend ostream& operator<<(ostream& os, const line3d& l);

	bool isDegenerated();

	double cosAngle(Vec3d v);

};