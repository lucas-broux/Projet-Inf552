#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "plane.hpp"
#include "line3dCloud.hpp"

using namespace std;
using namespace cv;

/**
* A class to calculate the ransac from pointcloud to find correlated planes or lines.
*/
class ransac {
private:
	int n_iterations;
	double epsilon;

public:
	/**
	* A constructor.
	* @param n_iterations The number of iterations for the algorithm.
	* @param epsilon Threshold.
	*/
	ransac(int n_iterations, double epsilon);

	/**
	* Extract the most correlated points (plane model).
	* @param pointCloud The considered point cloud.
	* @param uniformColor A boolean to tell if we want to have a uniform color or not.
	* @param color The color.
	* @return List of points that correlate the most (plane model) as point3dCloud.
	*/
	point3dCloud fit3dPlane(point3dCloud pointCloud, bool uniformColor = false, Vec3b color = Vec3b(0, 0, 0));

	/**
	* Extract the most correlated points (line model).
	* @param pointCloud The considered point cloud.
	* @param p The plane to be ~ orthogonal of.
	* @param uniformColor A boolean to tell if we want to have a uniform color or not.
	* @param color The color.
	* @param nlines The number of lines desired.
	* @param minDistBetweenLines The minimum distance between two lines (to avoid getting all the lines in the same space).
	* @return List of points that correlate the most (line model) as point3dCloud.
	*/
	point3dCloud fit3dLine(point3dCloud pointCloud, plane p, bool uniformColor = false, Vec3b color = Vec3b(0, 0, 0), int nlines = 1, double minDistBetweenLines = 0);
};