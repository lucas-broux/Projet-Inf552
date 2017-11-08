#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
using namespace std;
using namespace cv;

int main()
{
	Mat I1 = imread("../IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	Mat I2 = imread("../IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	//Mat I2 = imread("../IMG_0046r.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	//namedWindow("I1", 1);
	//namedWindow("I2", 1);
	//imshow("I1", I1);
	//imshow("I2", I2);

	vector<KeyPoint> m1, m2;
	Mat J1, J2;
	Ptr<AKAZE> akaze = AKAZE::create();
	akaze->detectAndCompute(I1, noArray(), m1, J1);
	akaze->detectAndCompute(I2, noArray(), m2, J2);

	Mat II1, II2;
	drawKeypoints(I1, m1, II1, Scalar::all(-1), 4);
	drawKeypoints(I2, m2, II2, Scalar::all(-1), 4);
	//imshow("I1", II1);
	//imshow("I2", II2);

	BFMatcher matcher(NORM_HAMMING);
	vector< vector<DMatch> > nn_matches;
	matcher.knnMatch(J1, J2, nn_matches, 2);

	Mat MATCHES;
	drawMatches(I1, m1, I2, m2, nn_matches, MATCHES);
	//imshow("MATCHES", MATCHES);

	vector<Point2f> matched1, matched2;
	for (size_t i = 0; i < nn_matches.size(); i++) {
		DMatch first = nn_matches[i][0];
		matched1.push_back(m1[first.queryIdx].pt);
		matched2.push_back(m2[first.trainIdx].pt);
	}

	Mat H = findHomography(matched1, matched2, CV_RANSAC);
	
	Mat K(I1.rows, 2*I1.cols,  CV_8U);
	warpPerspective(I2, K, H, K.size(), WARP_INVERSE_MAP);

	for (int i = 0; i < I1.rows; i++) {
		for (int j = 0; j < I1.cols; j++) {
			K.at<uchar>(i, j) = I1.at<uchar>(i, j);
		}
	}
	imshow("K", K);
	waitKey(0);
	return 0;
}
