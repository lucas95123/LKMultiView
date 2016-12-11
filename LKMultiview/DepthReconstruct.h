#pragma once
#include <iostream>
#include <string>

#include <opencv2\opencv.hpp>

#include "CvUtil.h"
#include "PhotoConsistency.h"

using namespace std;
using namespace cv;

Point2d projectPointWithDepth(const Matx33d &K, Point pt, Vec3d rvec, Vec3d tvec, Matx33d K2, double depth);

void calcDepthMapWinnerTakesAll(const vector<string> &vecImgNames, const vector<CameraParameter> &vecCameraParam, Mat &matDepth, int patchSize);