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

void printArray(const float *pfArr, int iCount)
{
	for (int i = 0; i < iCount; i++)
	{
		cout << pfArr[i] << " ";
	}
	cout << endl;
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
	Mat_<double> r = svd.u * Mat(W) * svd.vt;
	Mat_<double> t = svd.u.col(2);
	Matx34d P1(r(0, 0), r(0, 1), r(0, 2), t(0), r(1, 0), r(1, 1), r(1, 2), t(1),r(2, 0),r(2, 1), r(2, 2), t(2));

	cout << R << endl;
	cout << t << endl;
}
