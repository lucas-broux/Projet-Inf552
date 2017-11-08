#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

template <typename T> class Image : public Mat {
public:
	Image() {}
	Image(const Mat& A):Mat(A) {}
	Image(int m,int n,int type):Mat(m,n,type) {}
	inline T operator()(int i,int j) const { return at<T>(i,j); }
	inline T& operator()(int i,int j) { return at<T>(i,j); }
};

#endif