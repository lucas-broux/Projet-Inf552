#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

class product {
private:
	Vec3d v1;
	Vec3d v2;

public:

	product(Vec3d v1, Vec3d v2);

	double getScalar();

	Vec3d getVectorial();

};