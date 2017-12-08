#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "lib\json.hpp"

#include "point3dCloud.hpp"
#include "parameters.hpp"

using json = nlohmann::json;

using namespace std;
using namespace cv;

/**
* A class to represent the projectData for a situation.
*/
class projectData {
private:
	string filename;
	Matx33d cameraMatrix;
	Mat leftImage_blurred;
	Mat leftImage_unblurred;
	Mat disparity;

public:
	/**
	* A constructor.
	* @param filename The path to the file. It should be of the form : "something/aachen_000029_000019" and the constructor will manage to open the files with the good name and extensions.
	* @param disparityGaussianBlur Range of the Gaussian Blur. Equals to 3 by default.
	* @param left_imageGaussianBlur Range of the Gaussian Blur. Equals to 0 by default.
	*/
	projectData(string filename, int disparityGaussianBlur = 3, int left_imageGaussianBlur = 0);

	/**
	* A method to get the camera matrix.
	* @return The camera matrix.
	*/
	Matx33d getCameraMatrix();

	/**
	* A method to get the blurred left image.
	* @return The blurred  left image.
	*/
	Mat getLeftImageBlurred();

	/**
	* A method to get the unblurred left image.
	* @return The unblurred left image.
	*/
	Mat getLeftImageUnblurred();

	/**
	* A method to get the disparity.
	* @return The disparity.
	*/
	Mat getDisparity();

	/**
	* Generates a 3d point cloud from left image + disparity + transformation matrix.
	* Exports the result as .ply file.
	* @return The point cloud as vector<pair<Vec3d, Vec3b>>.
	*/
	point3dCloud pointCloudFromData();

};