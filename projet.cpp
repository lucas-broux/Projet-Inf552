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

int computeQ(Mat disparity) {
	// read a JSON file
	std::ifstream stream_camera("../Files/aachen_000029_000019_test/aachen_000029_000019_camera.json");
	json camera;
	stream_camera >> camera;

	Matx33d cameraMatrix;
	cameraMatrix(0, 0) = (double)camera["intrinsic"]["fx"];
	cameraMatrix(0, 1) = 0.0;
	cameraMatrix(0, 2) = (double)camera["intrinsic"]["u0"];
	cameraMatrix(1, 0) = 0.0;
	cameraMatrix(1, 1) = (double)camera["intrinsic"]["fy"];
	cameraMatrix(1, 2) = (double)camera["intrinsic"]["v0"];
	cameraMatrix(2, 0) = 0.0;
	cameraMatrix(2, 1) = 0.0;
	cameraMatrix(2, 2) = 1.0;
	//cout << cameraMatrix << endl;

	// Rotation + translation.
	Matx34d Rt;
	Rt(0, 0) = 1.0;
	Rt(0, 1) = 0.0;
	Rt(0, 2) = 0.0;
	Rt(0, 3) = (double)camera["extrinsic"]["baseline"];
	Rt(1, 0) = 0.0;
	Rt(1, 1) = 1.0;
	Rt(1, 2) = 0.0;
	Rt(1, 3) = 0.0;
	Rt(2, 0) = 0.0;
	Rt(2, 1) = 0.0;
	Rt(2, 2) = 1.0;
	Rt(2, 3) = 0.0;
	//cout << Rt << endl;
	//cout << cameraMatrix*Rt << endl;

	// Rotation.
	Matx33d R;
	R(0, 0) = 1.0;
	R(0, 1) = 0.0;
	R(0, 2) = 0.0;
	R(1, 0) = 0.0;
	R(1, 1) = 1.0;
	R(1, 2) = 0.0;
	R(2, 0) = 0.0;
	R(2, 1) = 0.0;
	R(2, 2) = 1.0;

	// Translation.
	Matx31d T;
	T(0, 0) = (double)camera["extrinsic"]["baseline"];
	T(1, 0) = 0.0;
	T(2, 0) = 0.0;

	Mat distortionCoefficients1 = (Mat1d(1, 4) << 0.0, 0.0, 0.0, 0.0);
	Mat distortionCoefficients2 = (Mat1d(1, 4) << 0.0, 0.0, 0.0, 0.0);

	Mat R1, R2, P1, P2, Q_cv;

	Size imageSize = disparity.size();

	cout << "R = " << R << endl;
	cout << "T = " << T << endl;
	cout << "cameraMatrix = " << cameraMatrix << endl;
	cout << "distortionCoefficients1 = " << distortionCoefficients1 << endl;
	cout << "distortionCoefficients2 = " << distortionCoefficients2 << endl;

	stereoRectify(cameraMatrix, distortionCoefficients1, cameraMatrix, distortionCoefficients2, imageSize, R, T, R1, R2, P1, P2, Q_cv);

	cout << "Q_cv = " << Q_cv << endl;

	//https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
	Mat_<float> Q(4, 4);
	Q.at<float>(0, 0) = 1.0;
	Q.at<float>(0, 1) = 0.0;
	Q.at<float>(0, 2) = 0.0;
	Q.at<float>(0, 3) = -(float)camera["intrinsic"]["u0"];
	Q.at<float>(1, 0) = 0.0;
	Q.at<float>(1, 1) = 1.0;
	Q.at<float>(1, 2) = 0.0;
	Q.at<float>(1, 3) = -(float)camera["intrinsic"]["v0"];
	Q.at<float>(2, 0) = 0.0;
	Q.at<float>(2, 1) = 0.0;
	Q.at<float>(2, 2) = 0.0;
	Q.at<float>(2, 3) = 2264.0;
	Q.at<float>(3, 0) = 0.0;
	Q.at<float>(3, 1) = 0.0;
	Q.at<float>(3, 2) = -1.0 / ((float)camera["extrinsic"]["baseline"]);
	Q.at<float>(3, 3) = 0.0;
	cout << "Q = " << Q << endl;
	return 0;
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

	Point m1(1050, 490);
	circle(left_image, m1, 1, Scalar(0, 255, 0), 1);
	//imshow("left_image", left_image);

	circle(right_image, m1, 1, Scalar(0, 255, 0), 1);
	Point m2(m1.x - float(disparity.at<uchar>(m1)) / 2.56, m1.y);
	circle(right_image, m2, 1, Scalar(0, 0, 255), 1);
	//imshow("right_image", right_image); waitKey();
	
	int counter = 0; // Count number of valid points.
	ofstream plyFile;// 3D Cloud.
	plyFile.open("../example.ply");
	// Header.
	plyFile << "ply\nformat ascii 1.0\ncomment author : Loiseau&Broux\ncomment object : 3d point Cloud\n";
	// Definition of element vertex.
	plyFile << "element vertex 8\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
	// Cube
	plyFile << "0 0 0 255 0 0\n";
	plyFile << "0 0 1 255 0 0\n";
	plyFile << "0 1 1 255 0 0\n";
	plyFile << "0 1 0 255 0 0\n";
	plyFile << "1 0 0 0 0 255\n";
	plyFile << "1 0 1 0 0 255\n";
	plyFile << "1 1 1 0 0 255\n";
	plyFile << "1 1 0 0 0 255\n";
	plyFile.close();
	Mat Diff(right_image.rows, right_image.cols, CV_32F);
	Mat isEqual(right_image.rows, right_image.cols, CV_32F);/*
	for (int i = 0; i < right_image.rows; i++) {
		for (int j = 0; j < right_image.cols; j++) {
			//cout << float(norm(right_image.at<Vec3b>(i, j) - left_image.at<Vec3b>(i, j))) << endl;
			if (i - float(disparity.at<uchar>(i, j)) / 2.56 >= 0) {
				if (float(norm(right_image.at<Vec3b>(i - float(disparity.at<uchar>(i, j)) / 2.56, j) - left_image.at<Vec3b>(i, j))) < 5) {
					isEqual.at<float>(i, j) = 255;

					if (float(disparity.at<uchar>(i, j) > 0)) {
						// Creer point 3D ICI
						float x = i;
						float y = j;
						Matx33d M;
						M(0, 0) = (double)camera["intrinsic"]["fx"];
						M(0, 1) = 0.0;
						M(0, 2) = (double)camera["intrinsic"]["u0"] - x;
						M(1, 0) = 0.0;
						M(1, 1) = (double)camera["intrinsic"]["fy"];
						M(1, 2) = (double)camera["intrinsic"]["v0"] - y;
						M(2, 0) = (double)camera["intrinsic"]["fx"];
						M(2, 1) = 0.0;
						M(2, 2) = (double)camera["intrinsic"]["u0"] - x - float(disparity.at<uchar>(i, j)) / 2.56;

						// Compute X, Y, Z.
						Mat tfx = (Mat1d(3, 1) << 0.0, 0.0, -(double)camera["intrinsic"]["fx"] * (double)camera["extrinsic"]["baseline"]);

						// Add point to file.

						// Increment counter.
						counter++;
						// cout << M.inv() * tfx << endl;
					}

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
	cout << "Counter : " << counter << endl;
	//imshow("isEqual", float2byte(isEqual));
	//imshow("Diff", float2byte(Diff)); waitKey();*/
	

	/*	
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