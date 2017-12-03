#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "include\json.hpp"

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
	Mat leftImage;
	Mat disparity;

public:
	/**
	* A constructor.
	* @param filename The path to the file. It should be like : "something/aachen_000029_000019" and the constructor will manage to open the files with the good name and extentions.
	* @param n Range of the Gaussian Blur. Equals to 2 by default.
	*/
	projectData(string filename, int gaussianBlur = 3);

	/**
	* A method to get the camera matrix.
	* @return The camera matrix.
	*/
	Matx33d getCameraMatrix();

	/**
	* A method to get the left image.
	* @return The left image.
	*/
	Mat getLeftImage();

	/**
	* A method to get the disparity.
	* @return The disparity.
	*/
	Mat getDisparity();

	/**
	Generates a 3d point cloud from left image + disparity + transformation matrix.
	Exports the result as .ply file.

	@param left_image The left image.
	@param disparity The disparity.
	@param N The matrix of correspondence : it can transform the disparity into 3d point.
	@return The point cloud as vector<pair<Vec3d, Vec3b>>.
	*/
	point3dCloud pointCloudFromData();

};