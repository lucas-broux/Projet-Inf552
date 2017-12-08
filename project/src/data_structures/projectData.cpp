#include "projectData.hpp"

projectData::projectData(string filename, int disparityGaussianBlur, int left_imageGaussianBlur) {
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

	// Read left image.
	Mat leftImage_unblurred = imread(filename + "_leftImg8bit.png");
	this->leftImage_unblurred = leftImage_unblurred;
	Mat leftImage = imread(filename + "_leftImg8bit.png");
	if (left_imageGaussianBlur != 0) {
		Mat leftImageBlured;
		GaussianBlur(leftImage, leftImageBlured, Size(left_imageGaussianBlur, left_imageGaussianBlur), 10);
		this->leftImage_blurred = leftImageBlured;
	}
	else {
		this->leftImage_blurred = leftImage;
	}

	// Read and smoothen disparity.
	// Smoothen disparity to have float values.
	Mat disparity_original = imread(filename + "_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	Mat disparity_float = imread(filename + "_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	Mat disparity;
	disparity_original.convertTo(disparity_float, CV_32FC1);
	GaussianBlur(disparity_float, disparity, Size(1, disparityGaussianBlur), 0.);
	this->disparity = disparity;
};


Matx33d projectData::getCameraMatrix() {
	return cameraMatrix;
};

Mat projectData::getLeftImageBlurred() {
	return leftImage_blurred;
};

Mat projectData::getLeftImageUnblurred() {
	return leftImage_unblurred;
};

Mat projectData::getDisparity() {
	return disparity;
};



/**
Determines if the pixel should be treated.

@param d The disparity between left/right images.
@return Whether the pixel should be considered.
*/
inline bool hasToBeTreated(int i, int j, double d, const Mat& left_image) {
	double xp = 1155.; double yp = 839.;
	// Remove car bonnet.
	if ((i > 1024. + j*(yp - 1024.) / xp) && (i > 1024. + (2048. - j)*(yp - 1024.) / (2048. - xp))) {
		return false;
	}
	// Disparity threshold.
	if (d < MIN_DISPARITY) {
		return false;
	}
	return true;
}


point3dCloud projectData::pointCloudFromData() {
	point3dCloud pointcloud; // The returned point3dCloud.

	// Loop over the image.
	for (int i = 0; i < leftImage_blurred.rows; i++) {
		for (int j = 0; j < leftImage_blurred.cols; j++) {
			double d = disparity.at<float>(i, j);
			if (hasToBeTreated(i, j, d, leftImage_blurred)) { // Adapt threshold for more/less 3d points.
													  // Compute coordinates + color of 3D point ( 1 matrix multiplication ).
				Mat pos_image = (Mat1d(3, 1) << i, j, 1.0);
				Vec3d position = (1 / d) * cameraMatrix * pos_image;
				Vec3b color = leftImage_blurred.at<Vec3b>(i, j);

				// Add point to point cloud.
				pointcloud.push_back(point3d(position, color, make_pair(i, j)));
			}
		}
	}

	// Return point3dCloud.
	return pointcloud;
};