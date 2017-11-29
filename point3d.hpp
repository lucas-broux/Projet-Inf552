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

class point3d {
private:
	Vec3d position;
	Vec3b color;

public:

	point3d(Vec3d position, Vec3b color);

	Vec3d getPosition();

	Vec3b getColor();
};