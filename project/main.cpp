/**
* This program intents to reconstruct a 3D scene
* from two images taken from a car in a street
* and to detect elements such as the road or the vertical objects
* @author Lucas Broux & Romain Loiseau
*/

#pragma once

#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "projectData.hpp"
#include "ransac.hpp"
#include "kMeans.hpp"
#include "parameters.hpp"

using namespace std;
using namespace cv;

int main(){

	/*---------------------
	|  1. INITIALIZATION  |
	---------------------*/
	// Access data.
	projectData data = projectData("../project/input/example_input/aachen_000029_000019", DISPARITY_GAUSSIAN_BLUR, LEFT_IMAGE_GAUSSIAN_BLUR);
	
	
	/*------------------------------------
	|  2. COMPUTATION OF 3D POINT CLOUD  |
	------------------------------------*/
	cout << "1. COMPUTATION OF 3D CLOUD." << endl;
	cout << "Computing... ";
	point3dCloud pointcloud = data.pointCloudFromData();
	cout << "Computed, extracted " << pointcloud.size() << " vertices." << endl;

	// Export point cloud as .ply file.
	cout << "Exporting point cloud as 3dcloud.ply ... ";
	pointcloud.pointCloud2ply("../project/output/main/3dcloud.ply");
	cout << "Exported." << endl;


	/*----------------------------
	|  3. DETECTION OF THE ROAD  |
	----------------------------*/
	cout << endl << "2. DETECTION OF THE ROAD." << endl;
	// Compute mean distance.
	cout << "Calculating mean distance in the point cloud ... ";
	double meanNeighboursDistance = pointcloud.meanNeighboursDistance();
	cout << "Mean neighbours distance in the point cloud = " << meanNeighboursDistance << endl;

	// Apply ransac.
	cout << "Applying ransac to find the road ... ";
	ransac rRoad = ransac(0.999, meanNeighboursDistance);
	pair<point3dCloud, point3dCloud> rRoadResult = rRoad.fit3dPlane(pointcloud, true, COLORS[RED]);
	
	// Apply regression.
	plane planeRoad;
	planeRoad.regression(rRoadResult.first);
	cout << "Ransac successfully applied, road plane equation: " << planeRoad << endl;
	
	// Show found plane on image.
	point3dCloud pointcloudRoad = rRoadResult.first;
	pointcloudRoad.showOnImage(data.getLeftImageUnblurred(), true, true, "../reports/images/Result_image_road.jpg");

	// Export result as .ply file.
	cout << "Exporting road point cloud as 3dcloud_road.ply ... ";
	pointcloudRoad.pointCloud2ply("../project/output/main/3dcloud_road.ply");
	cout << "Exported." << endl;


	/*--------------------------------------------------------
	|  4. DETECTION OF VERTICAL OBJECTS USING LINES + RANSAC |
	--------------------------------------------------------*/
	
	cout << endl << "3. DETECTION OF VERTICAL OBJECTS USING LINES + RANSAC." << endl;
	// Apply ransac.
	cout << "Applying ransac to find vertical objects...";
	ransac rVo = ransac(0.999, 2 * meanNeighboursDistance);
	point3dCloud pointcloudVO_ransac = rVo.fit3dLine(rRoadResult.second, planeRoad, true, COLORS[GREEN], 5, 4 * meanNeighboursDistance);

	// Export result as .ply file.
	cout << "Exporting result as .ply file...";
	pointcloudVO_ransac.pointCloud2ply("../project/output/main/3dcloud_verticalObjects.ply");
	cout << "Exported." << endl;

	// Show found vertical object on image.
	pointcloudVO_ransac.showOnImage(data.getLeftImageUnblurred(), true, true, "../reports/images/Result_image_vertical.jpg");


	/*----------------------------------------------
	|  5. DETECTION OF VERTICAL OBJECTS CLUSTERING |
	----------------------------------------------*/
	cout << endl << "4. DETECTION OF VERTICAL OBJECTS USING CLUSTERING." << endl;
	// Access data to get a clean version of the left image.
	data = projectData("../project/input/example_input/aachen_000029_000019", DISPARITY_GAUSSIAN_BLUR, LEFT_IMAGE_GAUSSIAN_BLUR);
	pointcloudRoad.showOnImage(data.getLeftImageUnblurred(), false);

	// Changing the base of coordinates to set the x axis as the altitude. It allows us to contract the altitude in the computation of kMeans.
	rRoadResult.second.changeBase(planeRoad.getABase());

	// Computing the standard deviations to analyse the data and reduce the color importance in the kMean computation.
	pair<Vec3d, Vec3d> sigmas = rRoadResult.second.getSigmas();
	double sigmaRatio = 0;
	if (norm(sigmas.second) != 0) {
		sigmaRatio = norm(sigmas.first) / norm(sigmas.second);
	}
	cout << "Sigma ratio = " << sigmaRatio << endl;

	// Computing the best number of cluster for the data using the silhouette score.
	int bestNumberOfClusters = 0;
	double bestScore = -1;
	for (int i = MIN_K_CLUSTERS; i < MAX_K_CLUSTERS; i++) {
		kMeans c = kMeans(i, sigmaRatio * KMEAN_COLOR_IMPORTANCE);
		int n_iterations = c.fit(rRoadResult.second);
		double score_i = c.computeScore();
		cout << "Score for " << i << " clusters = " << score_i << "           in " << n_iterations << " iterations." << endl;
		if (score_i > bestScore) {
			bestScore = score_i;
			bestNumberOfClusters = i;
		}
	}
	
	// Computing the kMeans.
	kMeans c = kMeans(bestNumberOfClusters);
	c.fit(rRoadResult.second);

	// Export result as .ply file.
	for (int i = 0; i < bestNumberOfClusters; i++) {
		cout << "Exporting point cloud as 3dcloud_cluster_" << i << ".ply for cluster " << i << " ... ";
		c.getClusters()[i].pointCloud2ply("../project/output/main/3dcloud_cluster"+to_string(i)+".ply");
		cout << "Exported." << endl;
	}

	// Show found vertical object on image.
	vector<point3dCloud> clusters = c.getClusters();
	for (int i = 0; i < clusters.size(); i++) {
		clusters[i].setColor(COLORS[(i+1)%COLORS.size()]);
		clusters[i].showOnImage(data.getLeftImageUnblurred(), i == clusters.size() - 1, i == clusters.size() - 1, "../reports/images/Result_image_vertical_clustering.jpg");
	}
	
	// End program.
	cout << endl << "End of program." << endl;
	int endofprogram;
	cin >> endofprogram;

	// Successfully exit program.
	return 0;
}