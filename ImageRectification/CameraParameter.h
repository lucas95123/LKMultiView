#pragma once
#include <opencv2\core.hpp>
#include <string>

using namespace std;
using namespace cv;

class CameraParameter
{
private:
	Mat K;
	Mat R;
	Mat T;
public:
	CameraParameter();
	CameraParameter(float *k, float *r, float *t);
	void setIntrinsicParam(float *k);
	Mat getIntrinsicParam();
	void setExtrinsicParamR(float *r);
	Mat getExtrinsicParamR();
	void setExtrinsicParamT(float *t);
	Mat getExtrinsicParamT();
	void print();
};

