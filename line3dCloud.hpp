#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "line3d.hpp"

using namespace std;
using namespace cv;

class line3dCloud {
private:
	vector<line3d> cloud;
	vector<int> npoints;

public:

	line3dCloud();

	void push_back(line3d line, int npoints);

	line3d operator[](int i);

	int size();

	int getMinNpointsIndex();

	int getMinNpoints();

	void set(int i, line3d line, int npoints);

	double minDistance(line3d line);

	double minDistance(Vec3d v);
};