#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "point3dCloud.hpp"

using namespace std;
using namespace cv;

class Plan {
private:
	double a, b, c, d; // a*x + b*y + c*z + d = 0

public:

	Plan();

	Plan(double a, double b, double c, double d);

	Plan(Vec3d p1, Vec3d p2, Vec3d p3);

	double distance(Vec3d p);

	void regression(point3dCloud pointcloud);

	friend ostream& operator<<(ostream& os, const Plan& p);

	bool isDegenerated();
};