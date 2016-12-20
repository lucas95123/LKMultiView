#pragma once
#include <iostream>
#include <string>
#include <opencv2\opencv.hpp>

#include "Patch.h"

using namespace std;
using namespace cv;

enum class Reconstruct
{
	WINNER_TAKES_ALL, OTHER
};

void calcDepthMapFromMultiView(
	Mat &matDepth,
	vector<string> &vecImgNames,
	Mat &K,
	vector<Mat> &vecRotation,
	vector<Mat> &vecTranslation,
	Reconstruct method,
	Metric metric,
	int patchSize = 5,
	float lower = 1.0,
	float upper = 11.0,
	float step = 1);

void calcDepthMapFromMultiViewWithMRF(
	Mat &matDepth,
	vector<string> &vecImgNames,
	Mat &K,
	vector<Mat> &vecRotation,
	vector<Mat> &vecTranslation,
	Reconstruct method,
	Metric metric,
	int patchSize = 5,
	float lower = 1.0,
	float upper = 11.0,
	float step = 1);