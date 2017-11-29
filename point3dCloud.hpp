#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "image.h"

#include "point3d.hpp"

using namespace std;
using namespace cv;

class point3dCloud {
private:
	vector<point3d> cloud;

public:

	point3dCloud();

	void push_back(point3d point);

	point3d operator[](int i);

	int size();

};