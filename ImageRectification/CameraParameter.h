#pragma once
#include <opencv2\core.hpp>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

class CameraParameter
{
private:
	bool m_isReference=false;

public:
	Matx33d K;
	Vec3d R;
	Vec3d T;

	CameraParameter();

	CameraParameter(double *k, double *r, double *t);

	void setIntrinsicParam(double *k);

	void setExtrinsicParamR(double *r);

	void setExtrinsicParamT(double *t);

	Mat getProjectionMatrix();

	void print();

	void setAsReference();

	bool isReference();

	bool isEmptyK() 
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				if (K(i, j) != 0.0)
					return false;
		return true;
	}
	bool isEmptyR() {
		for (int i = 0; i < 3; i++)
			if (R(i) != 0.0)
				return false;
		return true;
	}
	bool isEmptyT()
	{
		for (int i = 0; i < 3; i++)
			if (T(i) != 0.0)
				return false;
		return true;
	}

	void writeCameraPose(ofstream &fout);
};

