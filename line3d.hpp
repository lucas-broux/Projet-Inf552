#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "product.hpp"

using namespace std;
using namespace cv;

/**
* A class to represent lines in the 3d space. 
* We describe a line3d by one of its point and a direction vector. We are using Vec3d objects because we don't need the color argument to represent this explicit object.
*/
class line3d {
private:
	Vec3d point;
	Vec3d vector;

public:
	/**
	* A constructor.
	* This default constructor sets this->point and this->vector to zero.
	*/
	line3d();

	/**
	* A constructor.
	* This constructor allows us to define a line3d with either two points or a point and a vector.
	* @param point A point in the line3d.
	* @param vector A direction vector or a second point of the line3d.
	* @param second_Vec3d_isvector A boolean allowing us to call the constructor properly.
	*/
	line3d(Vec3d point, Vec3d v, bool second_Vec3d_isvector);

	/**
	* Finds the distance between the line3d and a point.
	* @param p The considered point.
	* @return The distance between the line3d and the considered point.
	*/
	double distance(Vec3d p);

	/**
	* Finds the distance between the line3d and another line3d.
	* @param l The considered line3d.
	* @return The distance between the two line3ds.
	*/
	double distance(line3d l);

	/**
	* Overloads ofstream for printing purposes.
	* @param os Considered stream.
	* @param l Considered line.
	* @return The ofstream to be printed.
	*/
	friend ostream& operator<<(ostream& os, const line3d& l);

	/**
	* Tels if the line is a correct line.
	* A line is degenerated if is direction vector is equal to zero.
	* @return A boolean telling if the line is a correct line.
	*/
	bool isDegenerated();

	/**
	* Gives the cosine between the line and a vector.
	* @param v The considered vector.
	* @return The cosine between the line and the vector.
	*/
	double cosAngle(Vec3d v);

};