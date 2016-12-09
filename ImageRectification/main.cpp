#include <iostream>
#include <fstream>

#include "Exceptions.h"
#include "CameraParameter.h"
#include "Util.h"
#include "CvUtil.h"
#include "KeyPointMatcher.h"
#include "Rectification.h"

#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
	string strDirPath = argv[1];
	int iRectNum = 5;
	if(argv[2]!=nullptr)
		iRectNum = str2int(argv[2]);

	vector<string> vecImgNames = getListFiles(strDirPath, "png", false);
	vector<CameraParameter> vecCameraParam;
	try {
		readCameraParameters(vecCameraParam, strDirPath+"/temple.txt");	
	}
	catch (BaseException e)
	{
		cout<<e.what()<<endl;
		exit(1);
	}

	//read the reference image
	Mat matImgRef=imread(vecImgNames[0]);

	//caculate RT between every frame and the reference frame
	for (int i = 1; i < vecImgNames.size(); i++)
	{
		//read the current image
		Mat matImgL, matImgR;
		matImgL = imread(vecImgNames[i - 1]);
		matImgR = imread(vecImgNames[i]);

		//find the correspondence with SIFT descriptor and RANSAC the result
		vector<pair<Point2f, Point2f>> matchPointPairsLR;
		getKeypointMatchesWithAllAlgorithms(matImgL, matImgR, matchPointPairsLR);

		//visualize matching results
		Mat matched = visualizeKeypointMatches(matImgL, matImgR, matchPointPairsLR);
		imwrite(strDirPath + "/matched_"+int2str(i-1)+"_"+int2str(i)+".jpg", matched);

		//convert an array of point pairs to two array of points
		vector<Point2f> matchedPtsL, matchedPtsR;
		pointPairToPoints(matchPointPairsLR, matchedPtsL, matchedPtsR);

		//find the essential matrix
		Mat E=findEssentialMat(matchedPtsL, matchedPtsR, vecCameraParam[i].K);
		
		//decompose the essential matrix to extract rotation and translation
		Mat R1, R2, t;
		decomposeEssentialMat(E, R1, R2, t);

		cout << R1 << endl;
		cout << R2 << endl;
		cout << t << endl;

		////calculate fundamental matrix using 8-point algorithm
		//Mat mask;
		//Mat F = findFundamentalMat(matchedPtsL, matchedPtsR, CV_FM_8POINT);
		//cout << "Fundamental matrix" << endl;
		//cout << F << endl;

		////save the rectified image for debugging
		//vector<pair<Point2f, Point2f>> warpedMatchPtsPairLR;
		//Mat rectImgL, rectImgR;
		//rectifyImagePair(matchedPtsL, matchedPtsR, F, matImgL, matImgR, rectImgL, rectImgR, warpedMatchPtsPairLR);
		//Mat matched1 = visualizeKeypointMatches(rectImgL, rectImgR, warpedMatchPtsPairLR);
		//imwrite(strDirPath + "/rectified_" + int2str(i - 1) + "_" + int2str(i) + ".jpg", matched1);

		////calculate essential matrix from fundamental matrix and camera intrinsics
		//Mat K1 = vecCameraParam[i - 1].K;
		//Mat K2 = vecCameraParam[i].K;
		//cout << "Intrinsic matrix" << endl;
		//cout << K1 << endl;
		//cout << K2 << endl;
		//Mat E = K2.t()*F*K1;
		//cout << "Essential matrix" << endl;
		//cout << E << endl;

		////decompse the essential matrix by SVD to get the rotation and translation between two camera
		//decomposeEssentialMatrix(E, vecCameraParam[i].R, vecCameraParam[i].T);
	}

	system("pause");
}