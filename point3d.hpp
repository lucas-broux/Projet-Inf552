#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

class point3d {
private:
	Vec3d position;
	Vec3b color;
	pair<int, int> pixel;

public:

	point3d(Vec3d position, Vec3b color, pair<int, int> pixel);

	Vec3d getPosition();

	Vec3b getColor();

	pair<int, int> getPixelCoordinates();

	double distance(point3d p);

};