#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "point3dCloud.hpp"

using namespace std;
using namespace cv;

/**
* A class to represent plane in the 3d space.
* We describe a plane by the four variables of its equation : a*x + b*y + c*z + d = 0.
*/
class plane {
private:
	double a, b, c, d; // a*x + b*y + c*z + d = 0

public:
	/**
	* A constructor.
	* This default constructor sets all the parameters to zero.
	*/
	plane();

	/**
	* A constructor.
	* @param a 1st parameter of a*x + b*y + c*z + d = 0
	* @param b 2nd parameter of a*x + b*y + c*z + d = 0
	* @param c 3rd parameter of a*x + b*y + c*z + d = 0
	* @param d 4th parameter of a*x + b*y + c*z + d = 0
	*/
	plane(double a, double b, double c, double d);

	/**
	* A constructor.
	* @param p1 1st point of the plane
	* @param p2 2nd point of the plane
	* @param p3 3rd point of the plane
	*/
	plane(Vec3d p1, Vec3d p2, Vec3d p3);

	/**
	* Finds the distance between the plane and a point.
	* @param p The considered point.
	* @return The distance between the plane and the considered point.
	*/
	double distance(Vec3d p);

	/**
	* Finds the closest plane to the given pointcloud.
	* This method modify the plane inplace.
	* @param pointcloud The considered pointcloud.
	*/
	void regression(point3dCloud pointcloud);

	/**
	* Overloads ofstream for printing purposes.
	* @param os Considered stream.
	* @param p Considered plane.
	* @return The ofstream to be printed.
	*/
	friend ostream& operator<<(ostream& os, const plane& p);

	/**
	* Tels if the plane is a correct plane.
	* @return A boolean telling if the plane is a correct plane.
	*/
	bool isDegenerated();

	/**
	* Gives a direction vector of the plane.
	* @return A direction vector of the plane.
	*/
	Vec3d getDirection();

};