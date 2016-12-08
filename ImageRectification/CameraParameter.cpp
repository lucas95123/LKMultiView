#include "CameraParameter.h"
#include "CvUtil.h"

CameraParameter::CameraParameter()
{

}

CameraParameter::CameraParameter(float * k, float * r, float * t)
{
	K = Mat(3, 3, CV_32FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K.at<float>(i, j) = k[i * 3 + j];
	R = Mat(3, 3, CV_32FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R.at<float>(i, j) = r[i * 3 + j];
	T = Mat(3, 1, CV_32FC1);
	for (int i = 0; i < 3; i++)
		T.at<float>(i, 0) = t[i];
}

void CameraParameter::setIntrinsicParam(float * k)
{
	K = Mat(3, 3, CV_32FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K.at<float>(i, j) = k[i * 3 + j];
}

Mat CameraParameter::getIntrinsicParam()
{
	return K;
}

void CameraParameter::setExtrinsicParamR(float * r)
{
	R = Mat(3, 3, CV_32FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R.at<float>(i, j) = r[i * 3 + j];
}

Mat CameraParameter::getExtrinsicParamR()
{
	return R;
}

void CameraParameter::setExtrinsicParamT(float * t)
{
	T = Mat(3, 1, CV_32FC1);
	for (int i = 0; i < 3; i++)
		T.at<float>(i, 0) = t[i];
}

Mat CameraParameter::getExtrinsicParamT()
{
	return T;
}

void CameraParameter::print()
{
	printMat(K);
	printMat(R);
	printMat(T);
}
