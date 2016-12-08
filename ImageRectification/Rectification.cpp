#include "Rectification.h"

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
	printMat(H1);
	printMat(H2);
	warpPerspective(imageL, rectifiedL, H1, Size(imageL.cols, imageL.rows), INTER_CUBIC);
	warpPerspective(imageR, rectifiedR, H2, Size(imageL.cols, imageL.rows), INTER_CUBIC);
	for (int i=0;i<ptsL.size();i++)
	{
		
		Mat vecHomoPt(3,1,CV_64FC1);
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
