#pragma once
#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <io.h>

#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

/*Data Type Conversion*/
int str2int(string str);
float str2float(string str);
double str2double(string str);
string int2str(int i);
string float2str(float f);
string double2str(double d);

/*IO*/
vector<string> getListFiles(
	string path,
	string extension,
	bool addPath
);

bool readCameraPoses(
	string path, 
	vector<Mat> &vecRotations, 
	vector<Mat> &vecTranslations
);

bool readCameraIntrinsics(
	string path,
	Mat &K
);

bool readDepthMap(
	string path,
	Mat &matDepth
);

/*Image Utility*/
void showImage(
	string windowName,
	const Mat &img,
	int delay = 1000,
	int mode=WINDOW_AUTOSIZE);

void normalizeAndRemapToGrayScale(
	Mat & matIn,
	Mat &matOut);

void visualizePCScores(const Mat&score, int step);

/*Math*/
float meanf(float* fPtr, int size);

float stdevf(float* fPtr, int size);

float devf(float * fPtr, int size);

float sumf(float * fPtr, int size);