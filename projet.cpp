#include <iostream>

#include <windows.h>

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

// Function for hiding/showing cursor : hiding with setcursror(0, 0); reinitialisation with setcursor(1, 10)
void setcursor(bool visible, DWORD size) // set bool visible = 0 - invisible, bool visible = 1 - visible
{
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	if (size == 0)
	{
		size = 20;	// default cursor size Changing to numbers from 1 to 20, decreases cursor width
	}
	CONSOLE_CURSOR_INFO lpCursor;
	lpCursor.bVisible = visible;
	lpCursor.dwSize = size;
	SetConsoleCursorInfo(console, &lpCursor);
}

int main()
{
	// Read JSON file containing camera info.
	std::ifstream stream_camera("../Files/aachen_000029_000019_test/aachen_000029_000019_camera.json");
	json camera;
	stream_camera >> camera;

	// Read the images
	Mat left_image = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_leftImg8bit.png");
	//Mat disparity = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0);


	// Smoothen disparity to have float values.
	Mat disparity_original = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0);
	Mat disparity_float = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0);
	Mat disparity;
	disparity_original.convertTo(disparity_float, CV_32FC1);
	GaussianBlur(disparity_float, disparity, Size(1, 9), 0.);

	int nb_vertex = 0; // Count number of valid points.
	ofstream plyFile;// 3D Cloud.
	plyFile.open("../3dcloud.ply");
	// Header.
	plyFile << "ply\nformat ascii 1.0\ncomment author : Loiseau & Broux\ncomment object : 3d point Cloud\n";
	// Definition of element vertex.
	string plyElements = "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
	string plyVertex;

	// Loop over the image.
	cout << "Looping over image :" << endl;
	for (int i = 0; i < left_image.rows; i++) {

		// Display progress bar.
		setcursor(0, 0); // Remove cursor in console.
		int nbarmax = 40;
		int nbar = (int)(i * nbarmax / left_image.rows) ;
		int pcent = (int)(i * 100 / left_image.rows);
		stringstream progressBar;
		progressBar << "[";
		for (int barCounter = 0; barCounter < nbarmax; barCounter++) {
			if (barCounter < nbar){
				progressBar << "|";
			}
			else {
				progressBar << " ";
			}
		}
		progressBar << "] " << pcent << "%" << "\r";;
		cout << progressBar.str();

		for (int j = 0; j < left_image.cols; j++) {
			double d = disparity.at<float>(i, j);
			if (d > 20) { // Adapt threshold for more/less 3d points.						
				// Compute coordinates of 3D point ( 1 matrix multiplication ).
				float x = i;
				float y = j;
				
				Matx33d N;
				N(0, 0) = 1.0;
				N(0, 1) = 0.0;
				N(0, 2) = -(double)camera["intrinsic"]["u0"];
				N(1, 0) = 0.0;
				N(1, 1) = (double)camera["intrinsic"]["fx"] / (double)camera["intrinsic"]["fy"];
				N(1, 2) = -(double)camera["intrinsic"]["fx"] * (double)camera["intrinsic"]["v0"] / (double)camera["intrinsic"]["fy"];
				N(2, 0) = 0.0;
				N(2, 1) = 0.0;
				N(2, 2) = (double)camera["intrinsic"]["fx"];
				N = -(double)camera["extrinsic"]["baseline"] / (d / 2.56) * N;
				Mat pos_image = (Mat1d(3, 1) << i, j, 1.0);
				Vec3d position = N * pos_image;

				// Add point to file.
				float X = position[0];
				float Y = position[1];
				float Z = position[2];
				Vec3b color = left_image.at<Vec3b>(i, j);
				int blue = color[0];
				int green = color[1];
				int red = color[2];
				stringstream currentString;
				currentString << X << " " << Y << " " << Z << " " << red << " " << green << " " << blue << endl;
				plyVertex.append(currentString.str());
				// Increment counter.
				nb_vertex++;
			}

		}
	}

	// Fill file.
	stringstream currentString;
	currentString << "element vertex " << nb_vertex << plyElements;
	plyElements = currentString.str();
	plyFile << plyElements << plyVertex;

	// Close file.
	plyFile.close();

	// Output result.
	system("cls");
	cout << "File exported : " << nb_vertex << " vertices extracted." << endl;
	
	return 0;
}