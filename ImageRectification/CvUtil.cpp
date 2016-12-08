#include "CvUtil.h"

void readCameraParameters(map<string, CameraParameter> &mapCameraParam, string path)
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

	float *k = new float[9];
	float *r = new float[9];
	float *t = new float[3];
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
		mapCameraParam[strImgName] = cameraParam;
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
