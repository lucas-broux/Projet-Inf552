#include <iostream>

#include <windows.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

/**
* A class to encapsulate the calculation of scalar and vectorial product.
*/
class product {
private:
	Vec3d v1;
	Vec3d v2;

public:
	/**
	* A constructor.
	* @param v1 The first vector.
	* @param v2 The second vector.
	*/
	product(Vec3d v1, Vec3d v2);

	/**
	* Computes the scalar product between the two points.
	* @return The scalar product.
	*/
	double getScalar();

	/**
	* Computes the vectorial product between the two points.
	* @return The vectorial product.
	*/
	Vec3d getVectorial();

};