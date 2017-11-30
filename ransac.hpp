#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "plane.hpp"
#include "line3dCloud.hpp"

using namespace std;
using namespace cv;

class ransac {
private:
	int n_iterations;
	double epsilon;

public:

	ransac(int n_iterations, double epsilon);

	point3dCloud fit3dPlane(point3dCloud pointCloud, bool uniformColor = false, Vec3b color = Vec3b(0, 0, 0));

	point3dCloud fit3dLine(point3dCloud pointCloud, plane p, bool uniformColor = false, Vec3b color = Vec3b(0, 0, 0), int nlines = 1, double minDistBetweenLines = 0);
};