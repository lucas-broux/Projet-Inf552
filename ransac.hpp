#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "plan.hpp"

using namespace std;
using namespace cv;

class Ransac {
private:
	int n_iterations;
	double epsilon;

public:

	Ransac(int n_iterations, double epsilon);

	point3dCloud fit(point3dCloud pointCloud);
};