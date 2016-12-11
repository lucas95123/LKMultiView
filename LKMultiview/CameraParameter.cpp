#include "CameraParameter.h"
#include "CvUtil.h"

CameraParameter::CameraParameter()
{
	Matx33d matR;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K(i, j) = 0;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matR(i, j) = 0;
	for (int i = 0; i<3; i++)
		T(i) = 0;
}

CameraParameter::CameraParameter(double * k, double * r, double * t)
{
	Matx33d matR;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K(i, j) = k[i * 3 + j];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matR(i, j) = r[i * 3 + j];
	Rodrigues(matR, R);
	for(int i=0;i<3;i++)
		T(i) = t[i];
}

void CameraParameter::setIntrinsicParam(double * k)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K(i, j) = k[i * 3 + j];
}

void CameraParameter::setExtrinsicParamR(double * r)
{
	Matx33d matR;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matR(i, j) = r[i * 3 + j];
	Rodrigues(matR, R);
}

void CameraParameter::setExtrinsicParamT(double * t)
{
	for (int i = 0; i < 3; i++)
		T(i) = t[i];
}

Matx34d CameraParameter::getProjectionMatrix()
{
	Mat_<double> matR;
	Rodrigues(R, matR);
	Matx34d matExtrinsix = Matx34d(matR(0, 0), matR(0, 1), matR(0, 2), T(0), matR(1, 0), matR(1, 1), matR(1, 2), T(1), matR(2, 0), matR(2, 1), matR(2, 2), T(2));
	return K*matExtrinsix;
}

void CameraParameter::print()
{
	cout << Mat(K);
	cout << Mat(R);
	cout << Mat(T);
}

void CameraParameter::setAsReference()
{
	Matx33d matR;
	m_isReference = true;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			if (i == j)
				matR(i, j) = 1.0;
			else
				matR(i, j) = 0.0;
	Rodrigues(matR, R);
	for (int i = 0; i < 3; i++)
		T(i) = 0.0;
}

bool CameraParameter::isReference()
{
	return m_isReference;
}

void CameraParameter::writeCameraPose(ofstream & fout)
{
	Mat matR;
	Rodrigues(R, matR);
	for (int i = 0; i < 9; i++)
		fout << matR.at<double>(i) << " ";
	for (int i = 0; i < 3; i++)
		fout << T(i) << " ";
	fout << endl;
}
