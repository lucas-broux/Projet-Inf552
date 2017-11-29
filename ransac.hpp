#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "image.h"

using namespace std;
using namespace cv;

class Ransac {
private:
	int n_iterations;
	double epsilon;

public:

	Ransac(int n_iterations, double epsilon);

	vector<pair<Vec3d, Vec3b>> fit(vector<pair<Vec3d, Vec3b>> pointCloud);
};