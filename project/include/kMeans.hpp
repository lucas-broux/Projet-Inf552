#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "point3dCloud.hpp"
#include "product.hpp"
#include "parameters.hpp"

using namespace std;
using namespace cv;

/**
* A class to calculate the ransac from pointcloud to find correlated planes or lines.
*/
class kMeans {
private:
	int k;
	double colorDilatationFactor;
	point3dCloud barycenters;
	vector<point3dCloud> clusters;

public:
	/**
	* A constructor.
	* @param k The number of clusters to be found.
	*/
	kMeans(int k);

	/**
	* A constructor.
	* @param k The number of clusters to be found.
	* @param colorDilatationFactor The color dilatation factor to set.
	*/
	kMeans(int k, double colorDilatationFactor);

	/**
	* Extracts the clusters for a point3dCloud.
	* @param pointCloud The considered point cloud.
	* @return The number of iterations the algorithm took to run.
	*/
	int fit(point3dCloud pointCloud);

	/**
	* Computes the clusters for a point3dCloud given the barycenters.
	* @param pointCloud The considered point cloud.
	*/
	void computeClusters(point3dCloud pointCloud);

	/**
	* Computes the barycenters for the given clusters.
	* @return The mean variation of position of the barycenters
	*/
	double computeBarycenters();

	/**
	* Initializas the barycenters by choosing random points in the point3dCloud.
	* @param pointCloud The considered point cloud.
	*/
	void initializeBarycenters(point3dCloud pointCloud);

	/**
	* A method to get the number of clusters.
	* @return The number of clusters.
	*/
	int getK();

	/**
	* A method to get the clusters.
	* @return The clusters.
	*/
	vector<point3dCloud> getClusters();

	/**
	* Computes the silhouette score.
	* @return The desired score.
	*/
	double computeScore();

	/**
	* Rescales the data.
	* @param vx The scaling factor on the x axis.
	* @param vy The scaling factor on the y axis.
	* @param vz The scaling factor on the z axis.
	*/
	void rescale(double vx, double vy, double vz);

	/**
	* Computes the distance between a point in a cluster and the barycenter of a cluster.
	* We use this distance to manage the computation of kMeans using positions and colors.
	* @param cluster_i The cluster in which is the point to consider.
	* @param point_i The point to consider.
	* @param barycenter_i The barycenter to consider.
	*/
	double distanceKMeans(int cluster_i, int point_i, int barycenter_i);
};