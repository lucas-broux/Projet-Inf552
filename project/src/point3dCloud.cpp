#include "point3dCloud.hpp"

point3dCloud::point3dCloud() {

};

void point3dCloud::push_back(point3d point) {
	cloud.push_back(point);
};

point3d point3dCloud::operator[](int i) {
	return(cloud[i]);
};

int point3dCloud::size() {
	return (int)cloud.size();
}

double point3dCloud::meanNeighboursDistance() {
	double d = 0;

	if (this->size() < 1000) {
		for (int i = 0; i < this->size(); i++) {
			double temp_d = DBL_MAX;
			for (int j = 0; j < this->size(); j++) {
				if (i != j && cloud[i].distance(cloud[j]) < temp_d) {
					temp_d = cloud[i].distance(cloud[j]);
				}
			}
			d += temp_d;
		}
		return(d / this->size());
	}
	else {
		for (int i = 0; i < 1000; i++) {
			int randomIndex1 = (rand()*RAND_MAX + rand()) % this->size();
			double temp_d = DBL_MAX;
			for (int j = 0; j < 10000; j++) {
				int randomIndex2 = (rand()*RAND_MAX + rand()) % this->size();
				if (cloud[randomIndex1].distance(cloud[randomIndex2]) < temp_d) {
					temp_d = cloud[randomIndex1].distance(cloud[randomIndex2]);
				}
			}
			d += temp_d;
		}
		return(d / 1000);
	}
}

void point3dCloud::showOnImage(Mat& image) {

	int i_max = image.rows;
	int j_max = image.cols;

	for (int point_counter = 0; point_counter < this->size(); point_counter ++) {
		pair<int, int> pixel = cloud[point_counter].getPixelCoordinates();
		int i = pixel.first;
		int j = pixel.second;
		if ((0 <= i) && (i < i_max) && (0 <= j) && (j < j_max)) {
			image.at<Vec3b>(i, j) = cloud[point_counter].getColor();
		}
	}
	Mat resized_image(512, 1024, image.depth());
	resize(image, resized_image, resized_image.size());
	//imwrite("../reports/images/Result_image.jpg", resized_image);
	imshow("Point cloud projection", resized_image); waitKey(0);
}

void point3dCloud::pointCloud2ply(string target) {
	// Define and open .ply file.
	ofstream plyFile;
	plyFile.open(target);
	// Write ply Header.
	plyFile << "ply\nformat ascii 1.0\ncomment author : Loiseau & Broux\ncomment object : 3d point Cloud\n";
	// Definition of element vertex.
	plyFile << "element vertex " << cloud.size() << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

	// Loop over 3d points.
	int n = cloud.size();
	for (int point_counter = 0; point_counter < n; point_counter++) {

		// Get point coordinates and color.
		Vec3d position = cloud[point_counter].getPosition();
		Vec3b color = cloud[point_counter].getColor();
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
};
