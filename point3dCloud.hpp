#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "point3d.hpp"

using namespace std;
using namespace cv;

/**
* A class to represent clouds of points in the 3d space.
* We describe a cloud by a vector of point3d.
*/
class point3dCloud {
private:
	vector<point3d> cloud;

public:

	/**
	* A constructor.
	* This default constructor initializes this->cloud.
	*/
	point3dCloud();

	/**
	* Adds a point in the pointcloud.
	* @param point The point3d to add.
	*/
	void push_back(point3d point);

	/**
	* Overloads [].
	* @param i The index to get.
	* @return The point3d.
	*/
	point3d operator[](int i);

	/**
	* Gives the size of the pointcloud.
	* @return The size of the pointcloud.
	*/
	int size();

	/**
	* Gives the mean of the distances between neighbours.
	* @return The mean.
	*/
	double meanNeighboursDistance();

	/**
	* Shows the result on an image.
	* This method allows us to see directly the result without visualizing a 3d pointcloud.
	*/
	void showOnImage(Mat& image);

};