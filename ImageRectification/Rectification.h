#pragma once
#include <iostream>
#include <string>

#include <opencv2\opencv.hpp>

#include "CvUtil.h"

using namespace std;
using namespace cv;

void rectifyImagePair(
	const vector<Point2f> &ptsL,
	const vector<Point2f> &ptsR,
	const Mat &F,
	const Mat &imageL,
	const Mat &imageR,
	Mat &rectifiedL,
	Mat &rectifiedR,
	vector<pair<Point2f, Point2f>> &wPtsLR);