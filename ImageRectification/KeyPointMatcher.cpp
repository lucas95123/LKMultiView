#include "KeyPointMatcher.h"

void getKeypointMatchesWithBRISK(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

	static const int kFlannMaxDistScale = 3;
	static const double kFlannMaxDistThreshold = 0.04;

	vector<KeyPoint> kptsL, kptsR;
	Mat descL, descR;
	vector<DMatch> goodMatches;

	Ptr<BRISK> brisk = BRISK::create();
	brisk->detect(imageL, kptsL);
	brisk->detect(imageR, kptsR);
	brisk->compute(imageL, kptsL, descL);
	brisk->compute(imageR, kptsR, descR);

	// FlannBasedMatcher with KD-Trees needs CV_32
	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	// KD-Tree param: # of parallel kd-trees
	static const int kFlannNumTrees = 4;
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = numeric_limits<float>::max();
	for (int i = 0; i < flannMatches.size(); ++i) {
		double dist = flannMatches[i].distance;
		maxDist = max(maxDist, dist);
		minDist = min(minDist, dist);
	}

	for (int i = 0; i < flannMatches.size(); ++i) {
		double distThresh = kFlannMaxDistScale * minDist;
		if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
			goodMatches.push_back(flannMatches[i]);
		}
	}

	for (const DMatch& match : goodMatches) {
		const Point2f& kptL = kptsL[match.queryIdx].pt;
		const Point2f& kptR = kptsR[match.trainIdx].pt;
		matchPointPairsLR.push_back(make_pair(kptL, kptR));
	}

	cout << "# matches from BRISK = " << goodMatches.size() <<endl;
}

void getKeypointMatchesWithORB(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

	static const int kFlannMaxDistScale = 3;
	static const double kFlannMaxDistThreshold = 0.04;

	vector<KeyPoint> kptsL, kptsR;
	Mat descL, descR;
	vector<DMatch> goodMatches;

	Ptr<ORB> orb = ORB::create();
	orb->detect(imageL, kptsL);
	orb->detect(imageR, kptsR);
	orb->compute(imageL, kptsL, descL);
	orb->compute(imageR, kptsR, descR);

	// FlannBasedMatcher with KD-Trees needs CV_32
	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	// KD-Tree param: # of parallel kd-trees
	static const int kFlannNumTrees = 4;
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = numeric_limits<float>::max();
	for (int i = 0; i < flannMatches.size(); ++i) {
		double dist = flannMatches[i].distance;
		maxDist = max(maxDist, dist);
		minDist = min(minDist, dist);
	}

	for (int i = 0; i < flannMatches.size(); ++i) {
		double distThresh = kFlannMaxDistScale * minDist;
		if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
			goodMatches.push_back(flannMatches[i]);
		}
	}

	for (const DMatch& match : goodMatches) {
		const Point2f& kptL = kptsL[match.queryIdx].pt;
		const Point2f& kptR = kptsR[match.trainIdx].pt;
		matchPointPairsLR.push_back(make_pair(kptL, kptR));
	}

	cout << "# matches from ORB = " << goodMatches.size() << endl;
}

void getKeypointMatchesWithSIFT(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

	static const int kFlannMaxDistScale = 3;
	static const double kFlannMaxDistThreshold = 0.04;

	Mat descL, descR;
	vector<KeyPoint> kptsL, kptsR;
	vector<DMatch> goodMatches;

	Ptr<SIFT> sift = SIFT::create();
	sift->detect(imageL, kptsL);
	sift->detect(imageR, kptsR);
	sift->compute(imageL, kptsL, descL);
	sift->compute(imageR, kptsR, descR);

	// FlannBasedMatcher with KD-Trees needs CV_32
	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	// KD-Tree param: # of parallel kd-trees
	static const int kFlannNumTrees = 4;
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = numeric_limits<float>::max();
	for (int i = 0; i < flannMatches.size(); ++i) {
		double dist = flannMatches[i].distance;
		maxDist = max(maxDist, dist);
		minDist = min(minDist, dist);
	}

	for (int i = 0; i < flannMatches.size(); ++i) {
		double distThresh = kFlannMaxDistScale * minDist;
		if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
			goodMatches.push_back(flannMatches[i]);
		}
	}

	for (const DMatch& match : goodMatches) {
		const Point2f& kptL = kptsL[match.queryIdx].pt;
		const Point2f& kptR = kptsR[match.trainIdx].pt;
		matchPointPairsLR.push_back(make_pair(kptL, kptR));
	}

	cout << "# matches from SIFT = " << goodMatches.size() <<endl;
}

void getKeypointMatchesWithAllAlgorithms(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

	vector<pair<Point2f, Point2f>> matchPointPairsLRAll;
	getKeypointMatchesWithSIFT(imageL, imageR, matchPointPairsLRAll);
	//getKeypointMatchesWithBRISK(imageL, imageR, matchPointPairsLRAll);
	//getKeypointMatchesWithORB(imageL, imageR, matchPointPairsLRAll);

	cout << "# matches total = " << matchPointPairsLRAll.size() <<endl;

	// TODO: remove duplicate keypoints

	// Apply RANSAC to filter weak matches (like, really weak)
	vector<Point2f> matchesL, matchesR;
	vector<uchar> inlinersMask;
	for (int i = 0; i < matchPointPairsLRAll.size(); ++i) {
		matchesL.push_back(matchPointPairsLRAll[i].first);
		matchesR.push_back(matchPointPairsLRAll[i].second);
	}

	static const int kRansacReprojThreshold = 100;
	findHomography(
		matchesL,
		matchesR,
		CV_RANSAC,
		kRansacReprojThreshold,
		inlinersMask);

	for (int i = 0; i < inlinersMask.size(); ++i) {
		if (inlinersMask[i]) {
			matchPointPairsLR.push_back(make_pair(matchesL[i], matchesR[i]));
		}
	}

	cout << "# matches after RANSAC = " << matchPointPairsLR.size() << endl;
}

Mat visualizeKeypointMatches(
	const Mat& imageL,
	const Mat& imageR,
	vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

	Mat visualization;
	hconcat(imageL, imageR, visualization);

	static const Scalar kVisPointColor = Scalar(110, 220, 0);
	for (auto& pointPair : matchPointPairsLR) {
		line(
			visualization,
			pointPair.first,
			pointPair.second + Point2f(imageL.cols, 0),
			kVisPointColor,
			1, // thickness
			CV_AA);
		circle(visualization, pointPair.first, 3, Scalar(0, 0, 255), 1, CV_AA);
		circle(visualization, pointPair.second + Point2f(imageL.cols, 0), 3, Scalar(0, 0, 255), 1, CV_AA);
	}
	return visualization;
}

void removeOutliers(const vector<Point2f>& matchedPtsL, const vector<Point2f>& matchedPtsR, vector<Point2f>& strongMatchPtsL, vector<Point2f>& strongMatchPtsR, const Mat & mask)
{
	for (int i = 0; i < matchedPtsL.size(); i++)
	{
		if (mask.at<uchar>(i) == 1)
		{
			strongMatchPtsL.push_back(matchedPtsL[i]);
			strongMatchPtsR.push_back(matchedPtsR[i]);
		}
	}
}

void removeOutliers(const vector<pair<Point2f,Point2f>>& matchedPtsLR, vector<pair<Point2f, Point2f>>& strongMatchPtsLR, const Mat & mask)
{
	for (int i = 0; i < matchedPtsLR.size(); i++)
	{
		if (mask.at<uchar>(i) == 1)
		{
			strongMatchPtsLR.push_back(matchedPtsLR[i]);
		}
	}
}

void rectifyImagePair(
	const vector<Point2f> &ptsL,
	const vector<Point2f> &ptsR,
	const Mat &F,
	const Mat &imageL,
	const Mat &imageR,
	Mat &rectifiedL,
	Mat &rectifiedR,
	vector<pair<Point2f, Point2f>> &wPtsLR)
{
	Mat H1, H2;
	stereoRectifyUncalibrated(ptsL, ptsR, F, Size(imageL.cols, imageL.rows), H1, H2);
	warpPerspective(imageL, rectifiedL, H1, Size(imageL.cols, imageL.rows), INTER_CUBIC);
	warpPerspective(imageR, rectifiedR, H2, Size(imageL.cols, imageL.rows), INTER_CUBIC);
	for (int i = 0; i<ptsL.size(); i++)
	{

		Mat vecHomoPt(3, 1, CV_64FC1);
		vecHomoPt.at<double>(0, 0) = ptsL[i].x;
		vecHomoPt.at<double>(1, 0) = ptsL[i].y;
		vecHomoPt.at<double>(2, 0) = 1.0f;
		Mat matResL = H1*vecHomoPt;
		double scale = matResL.at<double>(2, 0);
		Point2f ptResL(matResL.at<double>(0, 0) / scale, matResL.at<double>(1, 0) / scale);

		vecHomoPt.at<double>(0, 0) = ptsR[i].x;
		vecHomoPt.at<double>(1, 0) = ptsR[i].y;
		vecHomoPt.at<double>(2, 0) = 1.0f;
		Mat matResR = H2*vecHomoPt;
		scale = matResR.at<double>(2, 0);
		Point2f ptResR(matResR.at<double>(0, 0) / scale, matResR.at<double>(1, 0) / scale);

		wPtsLR.push_back(make_pair(ptResL, ptResR));
	}
}