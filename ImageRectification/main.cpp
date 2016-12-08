#include <iostream>
#include <fstream>

#include "Exceptions.h"
#include "CameraParameter.h"
#include "Util.h"
#include "CvUtil.h"
#include "KeyPointMatcher.h"
#include "Rectification.h"

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\contrib\contrib.hpp>
#include <opencv2\calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
	//string strDirPath = argv[1];
	//int iRectNum = 2;
	//if(argv[2]!=nullptr)
	//	iRectNum = str2int(argv[2]);

	//vector<string> vecImgNames = Directory::GetListFiles(strDirPath, "*.png");
	//map<string, CameraParameter> mapCameraParam;
	//try {
	//	readCameraParameters(mapCameraParam, "./templeSR_par.txt");	
	//}
	//catch (BaseException e)
	//{
	//	cout<<e.what()<<endl;
	//	exit(1);
	//}
	if (argc != 3)
	{
		cout << "Usage: ./ImageRectification img1_path img2_path" << endl;
		exit(0);
	}

	string strDirPath = argv[1];
	strDirPath = strDirPath.substr(0, strDirPath.find_last_of('\\'));

	//the first image as the reference image, rectify
	Mat matImgL, matImgR;
	matImgL = imread(argv[1]);
	matImgR = imread(argv[2]);

	//Finding Correspondence with SIFT descriptor and RANSAC the result
	vector<pair<Point2f, Point2f>> matchPointPairsLR;
	vector<pair<Point2f, Point2f>> strongMatchPtsLR;
	getKeypointMatchesWithAllAlgorithms(matImgL, matImgR, matchPointPairsLR);

	//Convert and array of point pairs to two array of points
	vector<Point2f> matchedPtsL, matchedPtsR;
	pointPairToPoints(matchPointPairsLR, matchedPtsL, matchedPtsR);

	//Calculate Fundamental matrix and remove outliers with RANSAC
	Mat mask;
	vector<Point2f> strongMatchedPtsL, strongMatchedPtsR;
	Mat F = findFundamentalMat(matchedPtsL, matchedPtsR, CV_FM_RANSAC,3.0,0.989999999999999999999999999, mask);
	removeOutliers(matchPointPairsLR, strongMatchPtsLR, mask);
	removeOutliers(matchedPtsL, matchedPtsR, strongMatchedPtsL, strongMatchedPtsR, mask);
	
	//Visualize matching results
	Mat matched = visualizeKeypointMatches(matImgL, matImgR, strongMatchPtsLR);
	imwrite(strDirPath+"/matched.png", matched);

	//Rectify image with Fundamental matrix
	Mat rectifiedL, rectifiedR;
	vector<pair<Point2f, Point2f>> retifiedPtsLR;
	rectifyImagePair(strongMatchedPtsL, strongMatchedPtsR, F, matImgL, matImgR, rectifiedL, rectifiedR, retifiedPtsLR);

	//Visualize matching results
	Mat matched1 = visualizeKeypointMatches(rectifiedL, rectifiedR, retifiedPtsLR);
	imwrite(strDirPath+"/matched_rectified.png", matched1);
}