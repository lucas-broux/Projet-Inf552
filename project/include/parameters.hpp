#pragma once
#include "color.hpp"


/*-------------------------------------------
|  COMPUTATION OF 3D POINT CLOUD PARAMETERS |
-------------------------------------------*/
const double MIN_DISPARITY = 10.; //The minimum disparity to be treated in projectData::pointCloudFromData().
const double DISPARITY_GAUSSIAN_BLUR = 3; //The gaussian blur range for the disparity.
const double LEFT_IMAGE_GAUSSIAN_BLUR = 25;//The gaussian blur range for the left image.


/*-------------------------
| RANSAC PLANE PARAMETERS |
-------------------------*/
const double APPROX_PROPORTION_OF_POINTS_IN_THE_PLANE = 0.2;


/*-------------------------
| RANSAC LINES PARAMETERS |
-------------------------*/
const double APPROX_PROPORTION_OF_POINTS_IN_A_LINE = 0.02;
const double MAX_ANGLE = 10.; //The maximum angle between the lines and the plane direction in ransac::fit3dLine.
const double PI = 3.14159265359;


/*-------------------
| KMEANS PARAMETERS |
-------------------*/
//The range in which will variate the number of clusters in our tests.
const double MIN_K_CLUSTERS = 2;
const double MAX_K_CLUSTERS = 10;

const double BARYCENTER_VARIATION_TRESHOLD = 0.01; //The treshold for mean barycenter variation in kMeans.
const double KMEAN_COLOR_IMPORTANCE = 0.75; //The importance of the color in the kMean algorithm.

//The factors we will use to rescale the pointclouds in kMeans.
const double SCALER_X = 0.5;
const double SCALER_Y = 1;
const double SCALER_Z = 1;


/*------------------
| OTHER PARAMETERS |
------------------*/
//The colors that will be used in visualization.
const vector<Vec3b> COLORS = color::defineColors();
const int RED = 0;
const int GREEN = 1;
const int BLUE = 2;
const int YELLOW = 3;
const int MAGENTA = 4;
const int CYAN = 5;