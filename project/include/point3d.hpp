#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

/**
* A class to represent points in the 3d space.
* We describe a point3d by its position, its color and its correponding pixel in the origin image.
*/
class point3d {
private:
	Vec3d position;
	Vec3b color;
	pair<int, int> pixel;

public:

	/**
	* A constructor.
	* @param position The position of the 3dPoint.
	* @param color The color of the 3dPoint.
	* @param pixel The corresponding pixel in the image.
	*/
	point3d(Vec3d position, Vec3b color, pair<int, int> pixel);

	/**
	* A method to get the position of the point.
	* @return The position of the point.
	*/
	Vec3d getPosition();

	/**
	* A method to get the color of the point.
	* @return The color of the point.
	*/
	Vec3b getColor();

	/**
	* A method to get the pixel coordinates of the point.
	* @return The pixel coordinates of the point.
	*/
	pair<int, int> getPixelCoordinates();

	/**
	* Finds the distance between the point and another point.
	* @param p The considered point.
	* @return The distance between the two points.
	*/
	double distance(point3d p);

	/**
	* Finds the distance between the point and a Vec3d.
	* @param p The considered Vec3d.
	* @return The distance between the two points.
	*/
	double distance(Vec3d v);

	/**
	* Overloads the equality of the points positions.
	* @param p The considered point3d.
	* @return A boolean telling if the two points are equals.
	*/
	bool operator==(point3d p);

	/**
	* Sets the color of the point.
	* @param color The desired color.
	*/
	void setColor(Vec3b color);

	/**
	* Sets the position of the point.
	* @param position The desired color.
	*/
	void setPosition(Vec3d position);

};