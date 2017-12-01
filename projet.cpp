/**
* This program intents to reconstruct a 3D scene
* from two images taken from a car in a street
* and to detect elements such as the road or the vertical objects
* @author Lucas Broux & Romain Loiseau
*/

#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "projectData.hpp"
#include "ransac.hpp"

using namespace std;
using namespace cv;


int main(){

	// Define data.
	projectData data = projectData("../files/aachen_000029_000019_test/aachen_000029_000019", 5);

	// Compute point cloud.
	Mat left_image = data.getLeftImage();
	Mat disparity = data.getDisparity();
	point3dCloud pointcloud = data.pointCloudFromData();

	// Export point cloud as .ply file.
	cout << "Exporting point cloud as 3dcloud.ply ... ";
	pointcloud.pointCloud2ply("../3dcloud.ply");
	cout << "Exported." << endl;

	//ROAD//
	cout << endl << "1. Finding the road." << endl;
	// Compute mean distance.
	cout << "Calculating mean distance in the point cloud ... ";
	double meanNeighboursDistance = pointcloud.meanNeighboursDistance();
	cout << "Mean neighbours distance in the point cloud = " << meanNeighboursDistance << endl;

	// Apply ransac.
	cout << "Applying ransac to find the road ... ";
	ransac rRoad = ransac(100, 2 * meanNeighboursDistance);
	point3dCloud pointcloudRoad = rRoad.fit3dPlane(pointcloud, true, Vec3b(0, 255, 0));
	
	// Apply regression.
	plane planeRoad;
	planeRoad.regression(pointcloudRoad);

	cout << "Ransac successfully applied, road plane equation: " << planeRoad << endl;

	// Export result as .ply file.
	cout << "Exporting road point cloud as 3dcloud_road.ply ... ";
	pointcloudRoad.pointCloud2ply("../3dcloud_road.ply");
	cout << "Exported." << endl;

	// Show found plane on image.
	pointcloudRoad.showOnImage(data.getLeftImage());

	//////Vertical objects
	cout << endl << "2. Finding vertical objects." << endl;
	// Apply ransac.
	cout << "Applying ransac to find vertical objects...";
	ransac rVo = ransac(1000, 10 * meanNeighboursDistance);
	point3dCloud pointcloudVo = rVo.fit3dLine(pointcloud, planeRoad, true, Vec3b(0, 0, 255), 3, 50 * meanNeighboursDistance);

	// Apply regression.
	//line3d line3dVo;
	//line3dVo.regression(pointcloudVo);

	// Export result as .ply file.
	cout << "Exporting result as .ply file...";
	pointcloudVo.pointCloud2ply("../3dcloud_verticalObjects.ply");
	cout << "Exported." << endl;

	// End program.
	cout << endl << "End of program." << endl;
	while (true) {

	}
	// Successfully exit file.
	return 0;
}