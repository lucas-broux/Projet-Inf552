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
#include "parameters.hpp"
#include "logger.hpp"

using namespace std;
using namespace cv;

// The number of inputs to process.
const int NUMBER_INPUTS = 57;

/**
	Converts clock value to time in seconds.

	@param t The considered clock.
	@return The time value in seconds.
*/
inline float clock2sec(clock_t t) {
	return ((float)(t)) / CLOCKS_PER_SEC;
}


/**
	Compute the average value of a float vector.

	@param vector The considered vector.
	@return The average value over the vector.
*/
inline float average(vector<float> vector) {
	return accumulate(vector.begin(), vector.end(), 0.0) / vector.size();
}


int main() {

	// Open logger. NB: DO NOT FORGET TO CLOSE AFTERWARDS.
	logger logger("../project/output/performance_evaluation/performance_report");

	// Get current time and do initial output.
	time_t _tm = time(NULL);
	struct tm * curtime = localtime(&_tm);
	logger.log_comment("Evaluation of performance.");
	logger.log_comment("Date of computation: " + (string)asctime(curtime));
	logger.log_comment("Global parameters used for computation:");
	logger.log_comment("\tDisparity Threshold: MIN_DISPARITY = " + to_string(MIN_DISPARITY));
	logger.log_comment("\tAngle Threshold: MIN_ANGLE = " + to_string(MIN_ANGLE) + "\n");

	// Initialize vector of times.
	vector<float> computation_time;
	vector<float> computation_time_3dcloud;
	vector<float> computation_time_road;
	vector<float> computation_time_verticalobjects;

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
		projectData data = projectData("../project/input/input_" + to_string(i) + "/leverkusen_" + num + "_000019", 3);

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
		computation_time_3dcloud.push_back(clock2sec(clock() - t_loc));
		t_loc = clock();
		


		/*----------------------------
		|  3. DETECTION OF THE ROAD  |
		----------------------------*/
		// Compute mean distance.
		double meanNeighboursDistance = pointcloud.meanNeighboursDistance();

		// Output result.
		logger.log_message(message(log_tag, i, "Mean Neighbours Distance", clock2sec(clock() - t_loc), "seconds"));
		logger.log_message(message(log_tag, i, "Distance", meanNeighboursDistance, ""));
		t_loc = clock();

		// Apply RANSAC.
		ransac rRoad = ransac(500, 2 * meanNeighboursDistance);
		point3dCloud pointcloudRoad = rRoad.fit3dPlane(pointcloud, true, Vec3b(0, 255, 0)).first;

		// Apply regression.
		plane planeRoad;
		planeRoad.regression(pointcloudRoad);

		// Output result.
		logger.log_message(message(log_tag, i, "Road detection (RANSAC)", clock2sec(clock() - t_loc), "seconds"));
		computation_time_road.push_back(clock2sec(clock() - t_loc));
		t_loc = clock();

		// Show found plane on image.
		pointcloudRoad.showOnImage(data.getLeftImage(), false);



		/*------------------------------------
		|  4. DETECTION OF VERTICAL OBJECTS  |
		------------------------------------*/
		// Apply RANSAC.
		ransac rVo = ransac(10000, 4 * meanNeighboursDistance);
		point3dCloud pointcloudVo = rVo.fit3dLine(pointcloud, planeRoad, true, Vec3b(0, 0, 255), 5, 9 * meanNeighboursDistance);

		// Output result.
		logger.log_message(message(log_tag, i, "Vertical object detection (RANSAC)", clock2sec(clock() - t_loc), "seconds"));
		computation_time_verticalobjects.push_back(clock2sec(clock() - t_loc));
		t_loc = clock();

		// Show found vertical object on image.
		pointcloudVo.showOnImage(data.getLeftImage(), false, true, "../project/output/performance_evaluation/leverkusen_" + num + "_000019.jpg");
		
		// Output global results for the iteration.
		t = clock() - t;
		logger.log_message(message(log_tag, i, "All data processing", clock2sec(t), "seconds"));
		computation_time.push_back(clock2sec(t));
	}
	
	// Compute and log average results.
	float average_computation_time = average(computation_time);
	float average_computation_time_3dcloud = average(computation_time_3dcloud);
	float average_computation_time_road = average(computation_time_road);
	float average_computation_time_verticalobjects = average(computation_time_verticalobjects);

	string log_tag = "[RESULTS]: ";
	logger.log_comment("[RESULTS]: Average computation time for 3d point cloud computation: " + to_string(average_computation_time_3dcloud) + " seconds.");
	logger.log_comment("[RESULTS]: Average computation time for road detection (RANSAC): " + to_string(average_computation_time_road) + " seconds.");
	logger.log_comment("[RESULTS]: Average computation time for vertical objects detection (RANSAC) : " + to_string(average_computation_time_verticalobjects) + " seconds.");
	logger.log_comment("[RESULTS]: Average computation time for data processing: " + to_string(average_computation_time) + " seconds.");

	// Close logger.
	logger.close();

	// End program.
	cout << endl << "End of program." << endl;
	int endofprogram;
	cin >> endofprogram;

	// Successfully exit program.
	return 0;
}