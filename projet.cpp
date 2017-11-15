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

Mat float2byte(const Mat& If) {
	double minVal, maxVal;
	minMaxLoc(If, &minVal, &maxVal);
	Mat Ib;
	If.convertTo(Ib, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
	return Ib;
}

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
	
	
	Mat Diff(right_image.rows, right_image.cols, CV_32F);
	Mat isEqual(right_image.rows, right_image.cols, CV_32F);
	for (int i = 0; i < right_image.rows; i++) {
		for (int j = 0; j < right_image.cols; j++) {
			//cout << float(norm(right_image.at<Vec3b>(i, j) - left_image.at<Vec3b>(i, j))) << endl;
			if (i - float(disparity.at<uchar>(i, j)) / 2.56 >= 0) {
				if (float(norm(right_image.at<Vec3b>(i - float(disparity.at<uchar>(i, j)) / 2.56, j) - left_image.at<Vec3b>(i, j))) == 0) {
					isEqual.at<float>(i, j) = 255;
				}
				else {
					isEqual.at<float>(i, j) = 0;
				}
				Diff.at<float>(i, j) = float(norm(right_image.at<Vec3b>(i - float(disparity.at<uchar>(i, j)) / 2.56, j) - left_image.at<Vec3b>(i, j)));
			}
			else {
				Diff.at<float>(i, j) = 0;
				isEqual.at<float>(i, j) = 0;
			}
		}
	}

	imshow("isEqual", float2byte(isEqual));
	imshow("Diff", float2byte(Diff)); waitKey();
	

	Point m1(1050, 490);
	circle(left_image, m1, 1, Scalar(0, 255, 0), 1);
	imshow("left_image", left_image);

	circle(right_image, m1, 1, Scalar(0, 255, 0), 1);
	Point m2(m1.x - float(disparity.at<uchar>(m1))/2.56, m1.y);
	circle(right_image, m2, 1, Scalar(0, 0, 255), 1);
	imshow("right_image", right_image); waitKey();
	
	/*
	//https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
	Matx44f Q;
	Q(0, 0) = 1.0;
	Q(0, 1) = 0.0;
	Q(0, 2) = 0.0;
	Q(0, 3) = (float) camera["intrinsic"]["u0"];
	Q(1, 0) = 0.0;
	Q(1, 1) = 1.0;
	Q(1, 2) = 0.0;
	Q(1, 3) = (float) camera["intrinsic"]["v0"];
	Q(2, 0) = 0.0;
	Q(2, 1) = 0.0;
	Q(2, 2) = 0.0;
	Q(2, 3) = 2264.0;
	Q(3, 0) = 0.0;
	Q(3, 1) = 0.0;
	Q(3, 2) = 1.0 / ((float)camera["extrinsic"]["baseline"]);
	Q(3, 3) = 0.0;
	cout << Q << endl;

	
	Mat xyz(disparity.size(), CV_32FC1);
	reprojectImageTo3D(disparity, xyz, Q, false);
	
	//https://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
	Mat_<Vec3f> XYZ(disparity.rows, disparity.cols);   // Output point cloud
	//Mat_<float> vec_tmp(4, 1);
	for (int y = 0; y<disparity.rows; ++y) {
		for (int x = 0; x<disparity.cols; ++x) {
			Vec4f vec_tmp(x, y, float(disparity.at<uchar>(y, x)),  1);
			if (y == 0 && x == 0) {
				cout << vec_tmp << endl;
			}
			vec_tmp = Q*vec_tmp;
			if (y == 0 && x == 0) {
				cout << vec_tmp << endl;
			}
			vec_tmp /= vec_tmp(3);
			if (y == 0 && x == 0) {
				cout << vec_tmp << endl;
			}
			Vec3f &point = XYZ.at<Vec3f>(y, x);
			point[0] = vec_tmp(0);
			point[1] = vec_tmp(1);
			point[2] = vec_tmp(2);
		}
	}
	
	cout << "projection in 3D finished" << endl;
	*/
	while (true);
	return 0;
}