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

/**
	Converts matrix of float values to bytes (float matrix to image conversion).

	@param If The float matrix.
	@return Ib The bytes matrix.
*/
Mat float2byte(const Mat& If) {
	double minVal, maxVal;
	minMaxLoc(If, &minVal, &maxVal);
	Mat Ib;
	If.convertTo(Ib, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
	return Ib;
}


/**
	Function for hiding/showing cursor : hiding with setcursror(0, 0); reinitialisation with setcursor(1, 10).

	@param visible Whether the cursor should be visible.
	@param size The size of the cursor.
*/
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

/**
	Determines if the pixel should be treated.

	@param d The disparity between left/right images.
	@return Whether the pixel should be considered.
*/
bool hasToBeTreated(int i, int j, double d, const Mat& left_image) {
	double xp = 1155.; double yp = 839.;
	if ((i > 1024. + j*(yp - 1024.)/xp) && (i > 1024. + (2048. - j)*(yp - 1024.) / (2048. - xp))) {
		return false;
	}
	if (d < 20) {
		return false;
	}
	return true;
}

/**
	Generates a 3d point cloud from left image + disparity + transformation matrix.
	Exports the result as .ply file.

	@param left_image The left image.
	@param disparity The disparity.
	@param N The matrix of correspondence : it can transform the disparity into 3d point.
*/
void pointCloudFromImages(Mat& left_image, const Mat& disparity, Matx33d N) {

	int nb_vertex = 0; // Count number of valid points.
	ofstream plyFile;// 3D Cloud.
	plyFile.open("../3dcloud.ply");
	// Write ply Header.
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
		int nbar = (int)(i * nbarmax / left_image.rows);
		int pcent = (int)(i * 100 / left_image.rows);
		stringstream progressBar;
		progressBar << "[";
		for (int barCounter = 0; barCounter < nbarmax; barCounter++) {
			if (barCounter < nbar) {
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
			if (hasToBeTreated(i, j, d, left_image)) { // Adapt threshold for more/less 3d points.
				// Compute coordinates of 3D point ( 1 matrix multiplication ).
				float x = i;
				float y = j;

				Mat pos_image = (Mat1d(3, 1) << i, j, 1.0);
				Vec3d position = (1 / d) * N * pos_image;

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
			else {
				Vec3b color;
				color[0] = 0;
				color[1] = 255;
				color[2] = 0;
				left_image.at<Vec3b>(i, j) = color;
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

	// Clear console and output result.
	system("cls");
	cout << "File exported : " << nb_vertex << " vertices extracted." << endl;

	Mat left_resized_image(512, 1024, left_image.depth());
	resize(left_image, left_resized_image, left_resized_image.size());
	imshow("left", left_resized_image); waitKey();
}

int main()
{
	// Read JSON file containing camera info and compute Matrix N of disparity correspondence.
	std::ifstream stream_camera("../Files/aachen_000029_000019_test/aachen_000029_000019_camera.json");
	json camera;
	stream_camera >> camera;
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
	N = -(double)camera["extrinsic"]["baseline"] * 2.56 * N;

	// Read the images
	Mat left_image = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_leftImg8bit.png");

	// Smoothen disparity to have float values.
	Mat disparity_original = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	Mat disparity_float = imread("../Files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0);
	Mat disparity;
	disparity_original.convertTo(disparity_float, CV_32FC1);
	GaussianBlur(disparity_float, disparity, Size(1, 5), 0.);

	// Compute point cloud.
	pointCloudFromImages(left_image, disparity, N);

	/*
	TODO: - Clean 3d point.
		  - Apply RANSAC to extract planes.
		  - Experiment with more efficient methods ?
	*/

	return 0;
}