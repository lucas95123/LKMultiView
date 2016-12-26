#pragma once
#include <iostream>
#include <string>
#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d.hpp>
#include <sba.h>

using namespace std;
using namespace cv;

void sparseBundleAdjustment();

void trackCamera(
	vector<string> &vecImgNames,
	Mat& K, 
	vector<Mat> &rotations, 
	vector<Mat> &transitions, 
	bool saveStructure=false,
	string saveDir=".\\");