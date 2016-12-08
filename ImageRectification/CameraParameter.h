#pragma once
#include <opencv2\core.hpp>
#include <string>

using namespace std;
using namespace cv;

class CameraParameter
{
public:
	Mat K;
	Mat R;
	Mat T;

	CameraParameter();
	CameraParameter(double *k, double *r, double *t);
	void setIntrinsicParam(double *k);
	Mat getIntrinsicParam();
	void setExtrinsicParamR(double *r);
	Mat getExtrinsicParamR();
	void setExtrinsicParamT(double *t);
	Mat getExtrinsicParamT();
	void print();
};

