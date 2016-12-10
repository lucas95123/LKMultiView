#include "CvUtil.h"

void readCameraParameters(vector<CameraParameter> &vecCameraParam, string path)
{
	ifstream fin(path);
	if (!fin.is_open())
	{
		throw FileNotFoundException("Camera parameter file does not exist");
	}

	int iSize;
	fin >> iSize;
	if (iSize<0)
	{
		throw InvalidFormatException("Invalid camera parameter file format");
	}

	double *k = new double[9];
	double *r = new double[9];
	double *t = new double[3];
	for (int i = 0; i < iSize; i++)
	{
		string strImgName;
		fin >> strImgName;

		for (int i = 0; i < 9; i++)
			fin >> k[i];
		for (int i = 0; i < 9; i++)
			fin >> r[i];
		for (int i = 0; i < 3; i++)
			fin >> t[i];

		CameraParameter cameraParam(k, r, t);
		vecCameraParam.push_back(cameraParam);
	}
	fin.close();
	delete[] k, r, t;
}

void writeMat(Mat & matData, string strPath)
{

}

void printArray(const float *pfArr, int iCount)
{
	for (int i = 0; i < iCount; i++)
	{
		cout << pfArr[i] << " ";
	}
	cout << endl;
}

void writeMat(ofstream & fout, const Mat & refMat)
{
	for (int i = 0; i < refMat.rows; i++)
	{
		for (int j = 0; j < refMat.cols; j++)
		{
			for(int k=0;k<3;k++)
				fout << refMat.at<Vec3f>(i, j)[k] << " ";
		}
		fout << endl;
	}
}

void printMat(const Mat &refMat)
{
	cout << refMat << endl;
}

void showImage(string windowName,const Mat &img)
{
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, img);
	waitKey(0);
}

void pointPairToPoints(const vector<pair<Point2f, Point2f>> &pairs, vector<Point2f> &points1, vector<Point2f> &points2)
{
	for (pair<Point2f, Point2f> p : pairs)
	{
		points1.push_back(p.first);
		points2.push_back(p.second);
	}
}

void decomposeEssentialMatrix(Mat &E, Mat &R, Mat&T)
{
	//perfrom SVD on E
	SVD svd = SVD(E);
	Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
	R = svd.u * Mat(W) * svd.vt;
	T = svd.u.col(2);
	//Matx34d P1(r(0, 0), r(0, 1), r(0, 2), t(0), r(1, 0), r(1, 1), r(1, 2), t(1),r(2, 0),r(2, 1), r(2, 2), t(2));

	cout << "Rotation Matrix" << endl;
	cout << R << endl;
	cout << "Transition Matrix" << endl;
	cout << T << endl;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return  norm(I, shouldBeIdentity) < 1e-6;

}

// Convert from cv::Mat to std::vector
void mat2vec4d(const Mat & matIn, vector<Vec4f>& vecOut, bool isRowMajor)
{
	if (isRowMajor)
	{
		for (int i = 0; i < matIn.rows; i++)
		{

			Vec4f P(matIn.at<float>(i, 0),
				matIn.at<float>(i, 1),
				matIn.at<float>(i, 2),
				matIn.at<float>(i, 3));
			vecOut.push_back(P);
		}
	}
	else
	{
		for (int i = 0; i < matIn.cols; i++)
		{

			Vec4f P(matIn.at<float>(0, i),
				matIn.at<float>(1, i),
				matIn.at<float>(2, i),
				matIn.at<float>(3, i));
			vecOut.push_back(P);
		}
	}
}

void vec2mat4d(const vector<Vec4f>& vecIn, Mat & matOut)
{
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

	assert(isRotationMatrix(R));

	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return Vec3f(x, y, z);
}
