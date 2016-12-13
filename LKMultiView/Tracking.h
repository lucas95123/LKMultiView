#pragma once
#include <iostream>
#include <string>
#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d.hpp>

using namespace std;
using namespace cv;

void trackCamera(
	vector<string> &vecImgNames,
	Mat& K, 
	vector<Mat> &rotations, 
	vector<Mat> &transitions, 
	bool saveStructure=false,
	string saveDir=".\\");