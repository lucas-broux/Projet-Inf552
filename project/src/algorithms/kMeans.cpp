#include "kMeans.hpp"

kMeans::kMeans(int k) {
	this->k = k;
	this->colorDilatationFactor = 0;
	this->barycenters = point3dCloud();
	this->clusters = vector<point3dCloud>();
};

kMeans::kMeans(int k, double colorDilatationFactor) {
	this->k = k;
	this->colorDilatationFactor = colorDilatationFactor;
	this->barycenters = point3dCloud();
	this->clusters = vector<point3dCloud>();
};

int kMeans::fit(point3dCloud pointCloud) {
	//Initializing barycenters by choosing them randomly in the pointCloud.
	this->initializeBarycenters(pointCloud);
	//Initializing the clusters.
	this->computeClusters(pointCloud);
	double deltaBarycenter_nMinus1 = DBL_MAX;
	double deltaBarycenter_n = DBL_MAX;
	int n_iterations = 0;
	//Iterations while deltaBarycenter variates less than BARYCENTER_VARIATION_TRESHOLD % in a step.
	while (n_iterations < 2 ||
		   deltaBarycenter_nMinus1 - deltaBarycenter_n > BARYCENTER_VARIATION_TRESHOLD * deltaBarycenter_nMinus1 ||
		   deltaBarycenter_nMinus1 < deltaBarycenter_n) {
		//Computing Barycenters
		deltaBarycenter_nMinus1 = deltaBarycenter_n;
		deltaBarycenter_n = this->computeBarycenters();
		//Initializing the clusters.
		this->computeClusters(pointCloud);
		n_iterations++;
	}
	return n_iterations;
}

void kMeans::computeClusters(point3dCloud pointCloud) {
	vector<point3dCloud> newClusters;
	for (int cluster_i = 0; cluster_i < k; cluster_i++) {
		newClusters.push_back(point3dCloud());
	}
	clusters[0] = pointCloud;
	for (int point_i = 0; point_i < pointCloud.size(); point_i++) {
		double minDistance = DBL_MAX;
		int bestBarycenter = 0;
		for (int barycenter_i = 0; barycenter_i < k; barycenter_i++) {
			double distance = this->distanceKMeans(0, point_i, barycenter_i);
			if (distance < minDistance) {
				minDistance = distance;
				bestBarycenter = barycenter_i;
			}
		}
		newClusters[bestBarycenter].push_back(pointCloud[point_i]);
	}
	clusters = newClusters;
}

double kMeans::computeBarycenters() {
	double deltaBarycenter = 0;
	point3dCloud newBarycenters;
	for (int barycenter_i = 0; barycenter_i < k; barycenter_i++) {
		Vec3d newBarycenterPosition = clusters[barycenter_i].getPositionBarycenter();
		if (colorDilatationFactor != 0) {
			Vec3b newBarycenterColor = clusters[barycenter_i].getColorBarycenter();
			deltaBarycenter += sqrt(pow(norm(product(Vec3d(SCALER_X, SCALER_Y, SCALER_Z), barycenters[barycenter_i].getPosition() - newBarycenterPosition).getTermToTerm()), 2) +
									pow(norm(colorDilatationFactor * (barycenters[barycenter_i].getColor() - newBarycenterColor)), 2));
			newBarycenters.push_back(point3d(newBarycenterPosition, newBarycenterColor, barycenters[barycenter_i].getPixelCoordinates()));
		}
		else {
			deltaBarycenter += norm(product(Vec3d(SCALER_X, SCALER_Y, SCALER_Z), barycenters[barycenter_i].getPosition() - newBarycenterPosition).getTermToTerm());
			newBarycenters.push_back(point3d(newBarycenterPosition, barycenters[barycenter_i].getColor(), barycenters[barycenter_i].getPixelCoordinates()));
		}
	}
	barycenters = newBarycenters;
	return deltaBarycenter / k;
};

void kMeans::initializeBarycenters(point3dCloud pointCloud) {
	point3dCloud iniBarycenters = point3dCloud();
	for (int barycenter_i = 0; barycenter_i < k; barycenter_i++) {
		int randomIndex = (rand()*RAND_MAX + rand()) % pointCloud.size();
		iniBarycenters.push_back(pointCloud[randomIndex]);
	}
	barycenters = iniBarycenters;
	clusters.push_back(pointCloud);
};

int kMeans::getK() {
	return k;
};

vector<point3dCloud> kMeans::getClusters() {
	return clusters;
};

double kMeans::computeScore() {
	double score = 0;
	int numPoints = 0;
	for (int cluster_i = 0; cluster_i < clusters.size(); cluster_i++) {
		numPoints += clusters[cluster_i].size();
		for (int point_i = 0; point_i < clusters[cluster_i].size(); point_i++) {
			double a = this->distanceKMeans(cluster_i, point_i, cluster_i);
			double b = DBL_MAX;
			for (int barycenter_i = 0; barycenter_i < clusters.size(); barycenter_i++){
				if (barycenter_i != cluster_i) {
					double temp_b = this->distanceKMeans(cluster_i, point_i, barycenter_i);
					if (temp_b < b) {
						b = temp_b;
					}
				}
			}
			score += (b - a) / max(a, b);
		}
	}
	return score / numPoints;
}

void kMeans::rescale(double vx, double vy, double vz) {
	for (int i = 0; i < clusters.size(); i++) {
		Vec3d newPosition = barycenters[i].getPosition();
		newPosition[0] *= vx;
		newPosition[1] *= vy;
		newPosition[2] *= vz;
		barycenters[i].setPosition(newPosition);
		for (int j = 0; j < clusters[i].size(); j++) {
			Vec3d newPosition = clusters[i][j].getPosition();
			newPosition[0] *= vx;
			newPosition[1] *= vy;
			newPosition[2] *= vz;
			clusters[i][j].setPosition(newPosition);
		}
	}
};

double kMeans::distanceKMeans(int cluster_i, int point_i, int barycenter_i) {
	double distance = norm(product(Vec3d(SCALER_X, SCALER_Y, SCALER_Z),	clusters[cluster_i][point_i].getPosition() - barycenters[barycenter_i].getPosition()).getTermToTerm());
	if (colorDilatationFactor != 0) {
		double distanceColor = norm(colorDilatationFactor * (barycenters[barycenter_i].getColor() - clusters[cluster_i][point_i].getColor()));
		distance = sqrt(pow(distance, 2) + pow(distanceColor, 2));
	}
	return distance;
};