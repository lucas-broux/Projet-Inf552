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

/**
	Function for hiding/showing cursor : hiding with setcursror(0, 0); reinitialisation with setcursor(1, 10).

	@param visible Whether the cursor should be visible.
	@param size The size of the cursor.
*/
void setcursor(bool visible, DWORD size) // set bool visible = 0 - invisible, bool visible = 1 - visible
{
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	if (size == 0)
	{
		size = 20;	// default cursor size Changing to numbers from 1 to 20, decreases cursor width
	}
	CONSOLE_CURSOR_INFO lpCursor;
	lpCursor.bVisible = visible;
	lpCursor.dwSize = size;
	SetConsoleCursorInfo(console, &lpCursor);
}

/**
	Determines if the pixel should be treated.

	@param d The disparity between left/right images.
	@return Whether the pixel should be considered.
*/
inline bool hasToBeTreated(int i, int j, double d, const Mat& left_image) {
	double xp = 1155.; double yp = 839.;
	// Remove car bonnet.
	if ((i > 1024. + j*(yp - 1024.)/xp) && (i > 1024. + (2048. - j)*(yp - 1024.) / (2048. - xp))) {
		return false;
	}
	// Disparity threshold.
	if (d < 20) {
		return false;
	}
	return true;
}


/**
	Generates .ply file from point cloud values.

	@param poincloud The corresponding point cloud.
*/
void pointCloud2ply(point3dCloud pointcloud, string target) {
	// Define and open .ply file.
	ofstream plyFile;
	plyFile.open(target);
	// Write ply Header.
	plyFile << "ply\nformat ascii 1.0\ncomment author : Loiseau & Broux\ncomment object : 3d point Cloud\n";
	// Definition of element vertex.
	plyFile << "element vertex " << pointcloud.size() << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

	// Loop over 3d points.
	int n = pointcloud.size();
	for (int point_counter = 0; point_counter < n; point_counter++) {

		// Get point coordinates and color.
		Vec3d position = pointcloud[point_counter].getPosition();
		Vec3b color = pointcloud[point_counter].getColor();
		double X = position[0];
		double Y = position[1];
		double Z = position[2];
		int blue = color[0];
		int green = color[1];
		int red = color[2];

		// Add point to file.
		plyFile << X << " " << Y << " " << Z << " " << red << " " << green << " " << blue << endl;
	}
	// Close file.
	plyFile.close();
}


/**
	Generates a 3d point cloud from left image + disparity + transformation matrix.
	Exports the result as .ply file.

	@param left_image The left image.
	@param disparity The disparity.
	@param N The matrix of correspondence : it can transform the disparity into 3d point.
	@return The point cloud as vector<pair<Vec3d, Vec3b>>.
*/
point3dCloud pointCloudFromImages(Mat& left_image, const Mat& disparity, Matx33d N) {

	point3dCloud pointcloud; // The returned vector.

	setcursor(0, 0); // Remove cursor in console.

	// Loop over the image.
	cout << "Looping over image :" << endl;
	for (int i = 0; i < left_image.rows; i++) {

		// Display progress bar.
		int nbarmax = 40;
		int nbar = (int)(i * nbarmax / left_image.rows);
		int pcent = (int)(i * 100 / left_image.rows);
		stringstream progressBar;
		progressBar << "[";
		for (int barCounter = 0; barCounter < nbarmax; barCounter++) {
			if (barCounter < nbar) {
				progressBar << "|";
			}
			else {
				progressBar << " ";
			}
		}
		progressBar << "] " << pcent << "%" << "\r";;
		cout << progressBar.str();

		for (int j = 0; j < left_image.cols; j++) {

			double d = disparity.at<float>(i, j);
			if (hasToBeTreated(i, j, d, left_image)) { // Adapt threshold for more/less 3d points.
				// Compute coordinates + color of 3D point ( 1 matrix multiplication ).
				Mat pos_image = (Mat1d(3, 1) << i, j, 1.0);
				Vec3d position = (1 / d) * N * pos_image;
				Vec3b color = left_image.at<Vec3b>(i, j);

				// Add point to point cloud.
				pointcloud.push_back(point3d(position, color, make_pair(i, j)));
			}
			else {
				// Color point on left image for vizualisation purposes.
				/*Vec3b color;
				color[0] = 0;
				color[1] = 255;
				color[2] = 0;
				left_image.at<Vec3b>(i, j) = color;*/
			}
		}
	}

	// Clear console and output result.
	system("cls");
	setcursor(1, 10);
	cout << "File exported : " << pointcloud.size() << " vertices extracted." << endl;

	// Show image.
	/*Mat left_resized_image(512, 1024, left_image.depth());
	resize(left_image, left_resized_image, left_resized_image.size());
	imshow("left", left_resized_image); waitKey();*/

	// Return point cloud.
	return pointcloud;

}


Matx33d computeCameraMatrix(string filename) {
	// Read JSON file containing camera info and compute Matrix N of disparity correspondence.
	Matx33d N;
	std::ifstream stream_camera(filename);
	json camera;
	stream_camera >> camera;

	N(0, 0) = 1.0;
	N(0, 1) = 0.0;
	N(0, 2) = -(double)camera["intrinsic"]["u0"];
	N(1, 0) = 0.0;
	N(1, 1) = (double)camera["intrinsic"]["fx"] / (double)camera["intrinsic"]["fy"];
	N(1, 2) = -(double)camera["intrinsic"]["fx"] * (double)camera["intrinsic"]["v0"] / (double)camera["intrinsic"]["fy"];
	N(2, 0) = 0.0;
	N(2, 1) = 0.0;
	N(2, 2) = (double)camera["intrinsic"]["fx"];
	N = -(double)camera["extrinsic"]["baseline"] * 2.56 * N;
	return N;
}

int main(){

	Matx33d N = computeCameraMatrix("../files/aachen_000029_000019_test/aachen_000029_000019_camera.json");
	cout << N << endl;
	/*
	// Read the images
	Mat left_image = imread("../files/aachen_000029_000019_test/aachen_000029_000019_leftImg8bit.png");

	// Smoothen disparity to have float values.
	Mat disparity_original = imread("../files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	Mat disparity_float = imread("../files/aachen_000029_000019_test/aachen_000029_000019_disparity.png", 0);
	Mat disparity;
	disparity_original.convertTo(disparity_float, CV_32FC1);
	GaussianBlur(disparity_float, disparity, Size(1, 5), 0.);
	*/

	projectData data = projectData("../files/aachen_000029_000019_test/aachen_000029_000019", 5);

	// Compute point cloud.
	Mat left_image = data.getLeftImage();
	cout << left_image.rows << " " << left_image.cols << " " << left_image.at<float>(0, 0) << endl;
	Mat disparity = data.getDisparity();
	cout << disparity.rows << " " << disparity.cols << " " << disparity.at<float>(0, 0) << endl;

	point3dCloud pointcloud = pointCloudFromImages(data.getLeftImage(), disparity, data.getCameraMatrix());

	cout << "Calculating mean distance in the point cloud" << endl;
	double meanNeighboursDistance = pointcloud.meanNeighboursDistance();
	cout << "Mean neighbours distance in the point cloud = " << meanNeighboursDistance << endl;

	cout << "Exporting as .ply file...";
	pointCloud2ply(pointcloud, "../3dcloud.ply");
	cout << "Exported." << endl;

	// Compute planee.
	/*Vec3d p1 = pointcloud[0].getPosition();
	Vec3d p2 = pointcloud[1523].getPosition();
	Vec3d p3 = pointcloud[28945].getPosition();
	cout << p1 << " " << p2 << " "<< p3 << endl;
	cout << "Computing planee" << endl;
	plane p = plane(p1, p2, p3);
	cout << p << endl;*/

	//////ROAD
	// Apply ransac.
	cout << "Applying ransac to find the road...";
	ransac rRoad = ransac(100, 2 * meanNeighboursDistance);
	point3dCloud pointcloudRoad = rRoad.fit3dPlane(pointcloud, true, Vec3b(0, 255, 0));
	
	// Apply regression.
	plane planeRoad;
	planeRoad.regression(pointcloudRoad);

	cout << planeRoad << " ransac successfully applied." << endl;

	// Export result as .ply file.
	cout << "Exporting result as .ply file...";
	pointCloud2ply(pointcloudRoad, "../3dcloud_road.ply");
	cout << "Exported." << endl;

	// Show found plane on image.
	pointcloudRoad.showOnImage(data.getLeftImage());

	//////Vertical objects
	// Apply ransac.
	cout << "Applying ransac to find vertical objects...";
	ransac rVo = ransac(1000, 10 * meanNeighboursDistance);
	point3dCloud pointcloudVo = rVo.fit3dLine(pointcloud, planeRoad, true, Vec3b(0, 0, 255), 3, 50 * meanNeighboursDistance);

	// Apply regression.
	//line3d line3dVo;
	//line3dVo.regression(pointcloudVo);

	// Export result as .ply file.
	cout << "Exporting result as .ply file...";
	pointCloud2ply(pointcloudVo, "../3dcloud_verticalObjects.ply");
	cout << "Exported." << endl;

	cout << endl;
	cout << "Programme termine" << endl;
	while (true) {

	}
	// Successfully exit file.
	return 0;
}