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
	string strDirPath = argv[1];
	int iRectNum = 5;
	if(argv[2]!=nullptr)
		iRectNum = str2int(argv[2]);

	vector<string> vecImgNames = Directory::GetListFiles(strDirPath, "*.png", false);
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
		matImgL = imread(strDirPath + "/" + vecImgNames[i - 1]);
		matImgR = imread(strDirPath + "/" + vecImgNames[i]);

		//find the correspondence with SIFT descriptor and RANSAC the result
		vector<pair<Point2f, Point2f>> matchPointPairsLR;
		getKeypointMatchesWithAllAlgorithms(matImgL, matImgR, matchPointPairsLR);

		//visualize matching results
		Mat matched = visualizeKeypointMatches(matImgL, matImgR, matchPointPairsLR);
		imwrite(strDirPath + "/matched_"+int2str(i-1)+"_"+int2str(i)+".jpg", matched);

		//convert an array of point pairs to two array of points
		vector<Point2f> matchedPtsL, matchedPtsR;
		pointPairToPoints(matchPointPairsLR, matchedPtsL, matchedPtsR);

		//calculate fundamental matrix using 8-point algorithm
		Mat mask;
		Mat F = findFundamentalMat(matchedPtsL, matchedPtsR, CV_FM_8POINT);

		//calculate essential matrix from fundamental matrix and camera intrinsics
		Mat K = vecCameraParam[i].K;
		Mat E = K.t()*F*K;

		decomposeEssentialMatrix(E, vecCameraParam[i].R, vecCameraParam[i].T);
	}

	system("pause");
}