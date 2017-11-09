#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "image.h"
#include "json.hpp"
using json = nlohmann::json;

using namespace std;
using namespace cv;

int main()
{
	cout << "Hello World" << endl;

	// read a JSON file
	std::ifstream stream_camera("../Files/aachen_000029_000019_test/aachen_000029_000019_camera.json");
	json camera;
	stream_camera >> camera;

	Matx33d cameraMatrix;
	cameraMatrix(0, 0) = (double) camera["intrinsic"]["fx"];
	cameraMatrix(0, 1) = 0.0;
	cameraMatrix(0, 2) = (double) camera["intrinsic"]["u0"];
	cameraMatrix(1, 0) = 0.0;
	cameraMatrix(1, 1) = (double) camera["intrinsic"]["fy"];
	cameraMatrix(1, 2) = (double) camera["intrinsic"]["v0"];
	cameraMatrix(2, 0) = 0.0;
	cameraMatrix(2, 1) = 0.0;
	cameraMatrix(2, 2) = 1.0;
	cout << cameraMatrix << endl;

	Matx34d Rt;
	Rt(0, 0) = 1.0;
	Rt(0, 1) = 0.0;
	Rt(0, 2) = 0.0;
	Rt(0, 3) = (double) camera["extrinsic"]["baseline"];
	Rt(1, 0) = 0.0;
	Rt(1, 1) = 1.0;
	Rt(1, 2) = 0.0;
	Rt(1, 3) = 0.0;
	Rt(2, 0) = 0.0;
	Rt(2, 1) = 0.0;
	Rt(2, 2) = 1.0;
	Rt(2, 3) = 0.0;
	cout << Rt << endl;
	cout << cameraMatrix*Rt << endl;

	Mat left_image = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_leftImg8bit.png");
	Mat right_image = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_rightImg8bit.png");
	Mat disparity = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_disparity.png");

	//https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
	Mat_<float> Q(4, 4);
	Q.at<float>(0, 0) = 1.0;
	Q.at<float>(0, 1) = 0.0;
	Q.at<float>(0, 2) = 0.0;
	Q.at<float>(0, 3) = (float) camera["intrinsic"]["u0"];
	Q.at<float>(1, 0) = 0.0;
	Q.at<float>(1, 1) = 1.0;
	Q.at<float>(1, 2) = 0.0;
	Q.at<float>(1, 3) = (float) camera["intrinsic"]["v0"];
	Q.at<float>(2, 0) = 0.0;
	Q.at<float>(2, 1) = 0.0;
	Q.at<float>(2, 2) = 0.0;
	Q.at<float>(2, 3) = 2264.0;
	Q.at<float>(3, 0) = 0.0;
	Q.at<float>(3, 1) = 0.0;
	Q.at<float>(3, 2) = 1.0 / ((float)camera["extrinsic"]["baseline"]);
	Q.at<float>(3, 3) = 0.0;

	/*
	Mat xyz(disparity.size(), CV_32FC3);
	reprojectImageTo3D(disparity, xyz, Q, false);
	*/

	//https://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
	Mat_<Vec3f> XYZ(disparity.rows, disparity.cols);   // Output point cloud
	Mat_<float> vec_tmp(4, 1);
	for (int y = 0; y<disparity.rows; ++y) {
		for (int x = 0; x<disparity.cols; ++x) {
			vec_tmp(0) = x; vec_tmp(1) = y; vec_tmp(2) = float(disparity.at<uchar>(y, x)); vec_tmp(3) = 1;
			vec_tmp = Q*vec_tmp;
			vec_tmp /= vec_tmp(3);
			Vec3f &point = XYZ.at<Vec3f>(y, x);
			point[0] = vec_tmp(0);
			point[1] = vec_tmp(1);
			point[2] = vec_tmp(2);
		}
	}

	cout << "projection in 3D finished" << endl;
	while (true);
	return 0;
}