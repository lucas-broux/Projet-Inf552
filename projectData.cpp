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

	// Read left image.
	Mat leftImage = imread(filename + "_leftImg8bit.png");
	this->leftImage = leftImage;

	// Read and smoothen disparity.
	// Smoothen disparity to have float values.
	Mat disparity_original = imread(filename + "_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	Mat disparity_float = imread(filename + "_disparity.png", 0); // Do not forget the 0 at the end for correct reading of the image.
	Mat disparity;
	disparity_original.convertTo(disparity_float, CV_32FC1);
	GaussianBlur(disparity_float, disparity, Size(1, gaussianBlur), 0.);
	this->disparity = disparity;
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
	if ((i > 1024. + j*(yp - 1024.) / xp) && (i > 1024. + (2048. - j)*(yp - 1024.) / (2048. - xp))) {
		return false;
	}
	// Disparity threshold.
	if (d < 20) {
		return false;
	}
	return true;
}


point3dCloud projectData::pointCloudFromData() {
	point3dCloud pointcloud; // The returned vector.

	// Loop over the image.
	cout << "Looping over image "  << filename  << " ... ";
	for (int i = 0; i < leftImage.rows; i++) {

		for (int j = 0; j < leftImage.cols; j++) {

			double d = disparity.at<float>(i, j);
			if (hasToBeTreated(i, j, d, leftImage)) { // Adapt threshold for more/less 3d points.
													  // Compute coordinates + color of 3D point ( 1 matrix multiplication ).
				Mat pos_image = (Mat1d(3, 1) << i, j, 1.0);
				Vec3d position = (1 / d) * cameraMatrix * pos_image;
				Vec3b color = leftImage.at<Vec3b>(i, j);

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

	// Output result.
	cout << "Point cloud generated: " << pointcloud.size() << " vertices extracted." << endl;

	// Show image.
	/*Mat left_resized_image(512, 1024, left_image.depth());
	resize(left_image, left_resized_image, left_resized_image.size());
	imshow("left", left_resized_image); waitKey();*/

	// Return point cloud.
	return pointcloud;
};