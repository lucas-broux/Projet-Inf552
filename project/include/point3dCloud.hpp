#pragma once

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
	* @return The caracteristic distance of the point cloud.
	*/
	double meanNeighboursDistance();

	/**
	* Shows the result on an image.
	* This method allows us to see directly the result without visualizing a 3d pointcloud.
	* @param image The corresponding image.
	* @param show A boolean telling if the image has to be shown.
	* @param save A boolean telling if the image has to be saved.
	* @param savepath The place where the file has to be saved.
	*/
	void showOnImage(Mat& image, bool show = true, bool save = false, string savepath = "");

	/**
	* Generates .ply file from point cloud values.
	* @param target The place where the file has to be saved.
	*/
	void pointCloud2ply(string target);

	/**
	* Gives the pointCloud without the points in the other.
	* As this method has a compexity in O(n^2), we will try to avoid using it.
	* @param poincloud The corresponding point cloud.
	*/
	point3dCloud deprivedOf(point3dCloud inputCloud);

	/**
	* Computes the barycenter of the positions of the points in the pointcloud.
	* @return The barycenter.
	*/
	Vec3d getPositionBarycenter();

	/**
	* Computes the barycenter of the colors of the points in the pointcloud.
	* @return The barycenter.
	*/
	Vec3b getColorBarycenter();

	/**
	* Sets the color of all the pointcloud to the desired color.
	* @param color The desired color.
	*/
	void setColor(Vec3b color);

	/**
	* Change the basis vectors of the pointcloud
	* @param P The tranfer matrix from the original to the new base.
	*/
	void changeBase(Mat P);

	/**
	* Computes the rangees in which evovles the positions and the color of the pointcloud.
	* @return The ranges in which evolves the positions and the color of the pointcloud as a pair<Vec6d, Vec6d>. The inferior boundaries are in the first element of the pair and the superior boundaries aire in the second.
	*/
	pair<Vec6d, Vec6d> getRanges();

	/**
	* Computes the standard deviations of the positions and the color of the pointcloud.
	* @return Thestandard deviations of the positions and the color of the pointcloud as a pair<Vec3d, Vec3d>.
	*/
	pair<Vec3d, Vec3d> getSigmas();
};