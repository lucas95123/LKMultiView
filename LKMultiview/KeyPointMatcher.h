#pragma once
#include <iostream>

#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace xfeatures2d;

void getKeypointMatchesWithBRISK(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR);

void getKeypointMatchesWithAKAZE(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR);

void getKeypointMatchesWithSIFT(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR);

// combines keypoint matches from BRISK, ORB, and AKAZE, and filters outliers with RANSAC
void getKeypointMatchesWithAllAlgorithms(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR);

// returns an image consisting of imageL and imageR stacked horizontally, with lines
// between matched keypoints
Mat visualizeKeypointMatches(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR);

// remove the outliers with a mask
void removeOutliers(
	const vector<Point2f> &matchedPtsL,
	const vector<Point2f> &matchedPtsR,
	vector<Point2f> &strongMatchPtsL,
	vector<Point2f> &strongMatchPtsR,
	const Mat &mask);

// remove the outliers with a mask, input in pairs
void removeOutliers(
	const vector<pair<Point2f, Point2f>>& matchedPtsLR,
	vector<pair<Point2f, Point2f>>& strongMatchPtsLR,
	const Mat & mask);

// rectify the image Pair with the fundamental matrix caculated by point correspondence
void rectifyImagePair(
	const vector<Point2f> &ptsL,
	const vector<Point2f> &ptsR,
	const Mat &F,
	const Mat &imageL,
	const Mat &imageR,
	Mat &rectifiedL,
	Mat &rectifiedR,
	vector<pair<Point2f, Point2f>> &wPtsLR);