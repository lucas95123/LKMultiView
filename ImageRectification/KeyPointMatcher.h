#pragma once
#include <iostream>

#include <opencv2\core.hpp>
#include <opencv2\calib3d.hpp>
#include <opencv2\features2d.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\nonfree\nonfree.hpp>

using namespace std;
using namespace cv;

void getKeypointMatchesWithBRISK(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR);

void getKeypointMatchesWithORB(
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