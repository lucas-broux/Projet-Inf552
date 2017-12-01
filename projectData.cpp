#include "projectData.hpp"

projectData::projectData(string filename, int gaussianBlur) {
	this->filename = filename;

	// Read JSON file containing camera info and compute Matrix N of disparity correspondence.
	Matx33d cameraMatrix;
	std::ifstream stream_camera(filename + "_camera.json");
	json camera;
	stream_camera >> camera;
	cameraMatrix(0, 0) = 1.0;
	cameraMatrix(0, 1) = 0.0;
	cameraMatrix(0, 2) = -(double)camera["intrinsic"]["u0"];
	cameraMatrix(1, 0) = 0.0;
	cameraMatrix(1, 1) = (double)camera["intrinsic"]["fx"] / (double)camera["intrinsic"]["fy"];
	cameraMatrix(1, 2) = -(double)camera["intrinsic"]["fx"] * (double)camera["intrinsic"]["v0"] / (double)camera["intrinsic"]["fy"];
	cameraMatrix(2, 0) = 0.0;
	cameraMatrix(2, 1) = 0.0;
	cameraMatrix(2, 2) = (double)camera["intrinsic"]["fx"];
	cameraMatrix = -(double)camera["extrinsic"]["baseline"] * 2.56 * cameraMatrix;
	this->cameraMatrix = cameraMatrix;

	// Read the images
	Mat leftImage = imread(filename + "_leftImg8bit.png");
	this->leftImage = leftImage;
	Mat disparity = imread(filename + "_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	this->disparity = disparity;
};

void projectData::smoothenDisparity(int n) {
	Mat disparity_float = imread(filename + "_disparity.png", 0);
	Mat actual_disparity = this->disparity;
	Mat disparity_smoothen;
	actual_disparity.convertTo(disparity_float, CV_32FC1);
	GaussianBlur(actual_disparity, disparity_smoothen, Size(1, n), 0.);
	this->disparity = disparity_smoothen;
};

Matx33d projectData::getCameraMatrix() {
	return cameraMatrix;
};

Mat projectData::getLeftImage() {
	return leftImage;
};

Mat projectData::getDisparity() {
	return disparity;
};