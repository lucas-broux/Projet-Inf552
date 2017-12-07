#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

/**
* A namespace to define the colors
*/
namespace color {
	vector<Vec3b> defineColors();
}