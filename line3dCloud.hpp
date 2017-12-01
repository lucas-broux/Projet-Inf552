#pragma once

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

/**
* A class to represent clouds of lines in the 3d space.
* We describe a cloud by a vector of line3d and a vector of int.
* The vector of int allows us to store the number of points next to the line from a pointcloud.
*/
class line3dCloud {
private:
	vector<line3d> cloud;
	vector<int> npoints;

public:
	/**
	* A constructor.
	* This default constructor initializes this->cloud and this->npoints.
	*/
	line3dCloud();

	/**
	* Adds a line in the linecloud.
	* @param line The line3d to add.
	* @param npoints The number of points next to the considered line.
	*/
	void push_back(line3d line, int npoints = 0);

	/**
	* Overloads [].
	* @param i The index to get.
	* @return The line3d.
	*/
	line3d operator[](int i);

	/**
	* Gives the size of the linecloud.
	* @return The size of the linecloud.
	*/
	int size();

	/**
	* Finds the index of the line with the least neighbours of the linecloud.
	* @return The desired index.
	*/
	int getMinNpointsIndex();

	/**
	* Finds the number of neighbours of the line with the least neighbours of the linecloud.
	* @return The desired number of neighbours.
	*/
	int getMinNpoints();

	/**
	* Replace a line for another in the linecloud.
	* @param i The index to change.
	* @param line The line3d.
	* @param npoints The number of neighbours of this line3d.
	*/
	void set(int i, line3d line, int npoints);

	/**
	* Finds distance of a line from it's closest in the pointcloud.
	* @param line The considered line.
	* @return The distance.
	*/
	double minDistance(line3d line);

	/**
	* Finds distance of a vector from it's closest line in the pointcloud.
	* @param v The considered vector.
	* @return The distance.
	*/
	double minDistance(Vec3d v);
};