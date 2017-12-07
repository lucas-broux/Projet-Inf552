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
			for (int j = 0; j < min(10000, this->size()); j++) {
				int randomIndex2 = (rand()*RAND_MAX + rand()) % this->size();
				if (randomIndex1 != randomIndex2 && cloud[randomIndex1].distance(cloud[randomIndex2]) < temp_d) {
					temp_d = cloud[randomIndex1].distance(cloud[randomIndex2]);
				}
			}
			d += temp_d;
		}
		return(d / 1000);
	}
}

void point3dCloud::showOnImage(Mat& image, bool show, bool save, string savepath) {

	int i_max = image.rows;
	int j_max = image.cols;

	for (int point_counter = 0; point_counter < this->size(); point_counter++) {
		pair<int, int> pixel = cloud[point_counter].getPixelCoordinates();
		int i = pixel.first;
		int j = pixel.second;
		if ((0 <= i) && (i < i_max) && (0 <= j) && (j < j_max)) {
			image.at<Vec3b>(i, j) = (cloud[point_counter].getColor() + 2 * image.at<Vec3b>(i, j)) / 3;
		}
	}
	Mat resized_image(512, 1024, image.depth());
	resize(image, resized_image, resized_image.size());

	if (save) {
		imwrite(savepath, resized_image);
	}
	if (show) {
		imshow("Point cloud projection", resized_image); waitKey(0);
	}
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

point3dCloud point3dCloud::deprivedOf(point3dCloud inputCloud) {
	point3dCloud outputCloud;

	for (int i = 0; i < inputCloud.size(); i++) {
		if (find(cloud.begin(), cloud.end(), inputCloud[i]) != cloud.end()) {
			/* cloud contains inputCloud[i] */
		}
		else {
			/* cloud does not contain inputCloud[i] */
			outputCloud.push_back(cloud[i]);
		}
	}

	return outputCloud;
};

Vec3d point3dCloud::getPositionBarycenter() {
	Vec3d b = Vec3d(0, 0, 0);
	for (int i = 0; i < this->size(); i++) {
		b += cloud[i].getPosition() / this->size();
	}
	return b;
};

Vec3b point3dCloud::getColorBarycenter() {
	Vec3b b = Vec3b(0, 0, 0);
	for (int i = 0; i < this->size(); i++) {
		b += cloud[i].getColor() / this->size();
	}
	return b;
};

void point3dCloud::setColor(Vec3b color) {
	for (int i = 0; i < this->size(); i++){
		cloud[i].setColor(color);
	}
}

void point3dCloud::changeBase(Mat P) {
	for (int i = 0; i < this->size(); i++) {
		Mat y(3, 1, CV_64FC1);
		y.at<double>(0, 0) = cloud[i].getPosition()[0];
		y.at<double>(1, 0) = cloud[i].getPosition()[1];
		y.at<double>(2, 0) = cloud[i].getPosition()[2];
		Mat Py = P * y;

 		Vec3d newPosition = Vec3d(Py.at<double>(0, 0), Py.at<double>(1, 0), Py.at<double>(2, 0));
		cloud[i].setPosition(newPosition);
	}
}

pair<Vec6d, Vec6d> point3dCloud::getRanges() {
	Vec6d infRanges = Vec6d(DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX);
	Vec6d supRanges = Vec6d(DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN);
	for (int i = 0; i < this->size(); i++) {
		for (int j = 0; j < 3; j++) {
			if (infRanges[j] > cloud[i].getPosition()[j]) {
				infRanges[j] = cloud[i].getPosition()[j];
			}
			if (infRanges[3 + j] > cloud[i].getColor()[j]) {
				infRanges[3 + j] = cloud[i].getColor()[j];
			}
			if (supRanges[j] < cloud[i].getPosition()[j]) {
				supRanges[j] = cloud[i].getPosition()[j];
			}
			if (supRanges[3 + j] < cloud[i].getColor()[j]) {
				supRanges[3 + j] = cloud[i].getColor()[j];
			}
		}
	}
	return pair<Vec6d, Vec6d>(infRanges, supRanges);
}

pair<Vec3d, Vec3d> point3dCloud::getSigmas() {
	Vec3d meanPosition = this->getPositionBarycenter();
	Vec3d meanColor = this->getColorBarycenter();

	Vec3d sigmaPosition = Vec3d(0, 0, 0);
	Vec3d sigmaColor = Vec3d(0, 0, 0);

	for (int i = 0; i < this->size(); i++) {
		for (int j = 0; j < 3; j++) {
			sigmaPosition[j] += pow(cloud[i].getPosition()[j] - meanPosition[j], 2) / this->size();
			sigmaColor[j] += pow(cloud[i].getColor()[j] - meanColor[j], 2) / this->size();
		}
	}

	for (int j = 0; j < 3; j++) {
		sigmaPosition[j] = sqrt(sigmaPosition[j]);
		sigmaColor[j] = sqrt(sigmaColor[j]);
	}

	return pair<Vec3d, Vec3d>(sigmaPosition, sigmaColor);
}