/**
* This program intents to reconstruct a 3D scene
* from two images taken from a car in a street
* and to detect elements such as the road or the vertical objects
* @author Lucas Broux & Romain Loiseau
*/

#include <iostream>

#include <windows.h>
#include <fstream>
#include <cmath>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "projectData.hpp"
#include "ransac.hpp"
#include "kMeans.hpp"
#include "parameters.hpp"
#include "logger.hpp"

using namespace std;
using namespace cv;


// The number of inputs to process.
const int NUMBER_INPUTS = 58;


/**
	Converts clock value to time in seconds.

	@param t The considered clock.
	@return The time value in seconds.
*/
inline float clock2sec(clock_t t) {
	return ((float)(t)) / CLOCKS_PER_SEC;
}


int main() {

	// Open logger. NB: DO NOT FORGET TO CLOSE AFTERWARDS.
	logger logger("../project/output/performance_evaluation/performance_report");

	// Get current time and do initial output.
	time_t _tm = time(NULL);
	struct tm * curtime = localtime(&_tm);
	logger.log_comment("Evaluation of performance.");
	logger.log_comment("Date of computation: " + (string)asctime(curtime));

	// Initialize clock.
	clock_t t, t_loc;

	// Loop over the data.
	for (int i = 0; i < NUMBER_INPUTS; i++) {

		/*----------------------------------
		|  1. INITIALIZATION OF ITERATION  |
		----------------------------------*/
		// Output information.
		logger.log_comment("Input data from ../project/input/input_" + to_string(i));

		// Access data.
		string num; // Number padded with 0s in front of it.
		if (i == 0) num = "000000";
		else {
			num = to_string(i);
			for (int num0 = 0; num0 < log10(100000 / i); num0++) num = "0" + num;
		}
		projectData data = projectData("../project/input/input_" + to_string(i) + "/leverkusen_" + num + "_000019", DISPARITY_GAUSSIAN_BLUR, LEFT_IMAGE_GAUSSIAN_BLUR);

		// Get current time.
		t = clock();
		t_loc = clock();



		/*------------------------------------
		|  2. COMPUTATION OF 3D POINT CLOUD  |
		------------------------------------*/
		// Compute point cloud.
		point3dCloud pointcloud = data.pointCloudFromData();

		// Output result.
		string log_tag = "[INPUT " + to_string(i) + "]";
		logger.log_message(message(log_tag, i, "3D Point cloud", clock2sec(clock() - t_loc), "seconds"));
		logger.log_message(message(log_tag, i, "Vertices", pointcloud.size(), "vertices"));
		t_loc = clock();
		


		/*--------------------------------------
		|  3. DETECTION OF THE ROAD  (RANSAC)  |
		--------------------------------------*/
		// Compute mean distance.
		double meanNeighboursDistance = pointcloud.meanNeighboursDistance();

		// Output result.
		logger.log_message(message(log_tag, i, "Mean Neighbours Distance", clock2sec(clock() - t_loc), "seconds"));
		logger.log_message(message(log_tag, i, "Distance", meanNeighboursDistance, ""));
		t_loc = clock();

		// Apply RANSAC.
		ransac rRoad = ransac(0.999, meanNeighboursDistance);
		pair<point3dCloud, point3dCloud> rRoadResult = rRoad.fit3dPlane(pointcloud, true, COLORS[RED]);


		// Apply regression.
		plane planeRoad;
		planeRoad.regression(rRoadResult.first);

		// Output result.
		logger.log_message(message(log_tag, i, "Road detection (RANSAC)", clock2sec(clock() - t_loc), "seconds"));
		t_loc = clock();

		// Show found plane on image.
		(rRoadResult.first).showOnImage(data.getLeftImage(), false);



		/*---------------------------------------------
		|  4. DETECTION OF VERTICAL OBJECTS (RANSAC)  |
		---------------------------------------------*/
		// Apply RANSAC.
		ransac rVo = ransac(0.999, 4 * meanNeighboursDistance);
		point3dCloud pointcloudVO_ransac = rVo.fit3dLine(rRoadResult.second, planeRoad, true, COLORS[GREEN], 5, 9 * meanNeighboursDistance);

		// Output result.
		logger.log_message(message(log_tag, i, "Vertical object detection (RANSAC)", clock2sec(clock() - t_loc), "seconds"));
		t_loc = clock();

		// Show found vertical object on image.
		pointcloudVO_ransac.showOnImage(data.getLeftImage(), false, true, "../project/output/performance_evaluation/images/ransac/leverkusen_" + num + "_000019.jpg");
		


		/*-------------------------------------------------
		|  5. DETECTION OF VERTICAL OBJECTS (CLUSTERING)  |
		-------------------------------------------------*/
		
		// Access data to get a clean version of the left image.
		data = projectData("../project/input/input_" + to_string(i) + "/leverkusen_" + num + "_000019", DISPARITY_GAUSSIAN_BLUR, LEFT_IMAGE_GAUSSIAN_BLUR);

		// Changing the base of coordinates to set the x axis as the altitude. It allows us to contract the altitude in the computation of kMeans.
		(rRoadResult.second).changeBase(planeRoad.getABase());

		// Computing the standard deviations to analyse the data and reduce the color importance in the kMean computation.
		pair<Vec3d, Vec3d> sigmas = (rRoadResult.second).getSigmas();
		double sigmaRatio = 0;
		if (norm(sigmas.second) != 0) {
			sigmaRatio = norm(sigmas.first) / norm(sigmas.second);
		}

		// Computing the best number of cluster for the data using the silhouette score.
		int bestNumberOfClusters = 0;
		double bestScore = -1;
		for (int i = MIN_K_CLUSTERS; i < MAX_K_CLUSTERS; i++) {
			kMeans c = kMeans(i, sigmaRatio * KMEAN_COLOR_IMPORTANCE);
			int n_iterations = c.fit(rRoadResult.second);
			double score_i = c.computeScore();
			if (score_i > bestScore) {
				bestScore = score_i;
				bestNumberOfClusters = i;
			}
		}

		// Computing the kMeans.
		kMeans c = kMeans(bestNumberOfClusters);
		c.fit(rRoadResult.second);

		// Output result.
		logger.log_message(message(log_tag, i, "Vertical object detection (Clustering)", clock2sec(clock() - t_loc), "seconds"));
		logger.log_message(message(log_tag, i, "Number of clusters", bestNumberOfClusters, ""));

		// Show found vertical object on image.
		vector<point3dCloud> clusters = c.getClusters();
		for (int i = 0; i < clusters.size(); i++) {
			clusters[i].setColor(COLORS[(i + 1) % COLORS.size()]);
			clusters[i].showOnImage(data.getLeftImage(), false, i == clusters.size() - 1, "../project/output/performance_evaluation/images/clustering/leverkusen_" + num + "_000019.jpg");
		}
		


		/*------------------------------
		|  6. CONCLUSION OF ITERATION  |
		------------------------------*/

		// Output global results for the iteration.
		t = clock() - t;
		logger.log_message(message(log_tag, i, "All data processing", clock2sec(t), "seconds"));

	}
	
	// Close logger.
	logger.close();

	// End program.
	cout << endl << "End of program." << endl;
	int endofprogram;
	cin >> endofprogram;

	// Successfully exit program.
	return 0;
}
