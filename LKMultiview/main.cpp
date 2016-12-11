#include <iostream>
#include <fstream>

#include "Exceptions.h"
#include "CameraParameter.h"
#include "Util.h"
#include "CvUtil.h"
#include "KeyPointMatcher.h"
#include "PhotoConsistency.h"
#include "DepthReconstruct.h"

#include <opencv2\opencv.hpp>

#define M_PI 3.1415926536

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
	string strDirPath = argv[1];
	int iRectNum = 5;
	if(argv[2]!=nullptr)
		iRectNum = str2int(argv[2]);
	
	bool bHaveCampos;

	vector<string> vecImgNames = getListFiles(strDirPath, "png", false);
	vector<CameraParameter> vecCameraParam;

	try {
		readCameraParameters(vecCameraParam, strDirPath+"/camparam.txt");
	}
	catch (BaseException e)
	{
		cout<<e.what()<<endl;
		return 0;
	}

	try {
		readCameraPose(vecCameraParam, strDirPath + "/campos.txt");
		bHaveCampos = true;
	}
	catch (BaseException e)
	{
		cout << e.what() << endl;
		bHaveCampos = false;
	}
	if (!bHaveCampos)
	{
		//read the reference image
		Mat matImgRef = imread(vecImgNames[0]);

		//set the first image as the reference image
		vecCameraParam[0].setAsReference();

		//campos file
		ofstream fout(strDirPath + "/campos.txt");
		fout << iRectNum << endl;
		fout << vecImgNames[0].substr(vecImgNames[0].find_last_of('\\') + 1) << " ";
		vecCameraParam[0].writeCameraPose(fout);

		//caculate RT between every frame and the reference frame
		for (int i = 1; i < vecImgNames.size() && i < iRectNum; i++)
		{
			//read the current image
			Mat matImgL, matImgR;
			matImgL = imread(vecImgNames[i - 1]);
			matImgR = imread(vecImgNames[i]);

			//find the correspondence with SIFT descriptor and RANSAC the result
			vector<pair<Point2f, Point2f>> vecMatchPointPairsLR;
			getKeypointMatchesWithAllAlgorithms(matImgL, matImgR, vecMatchPointPairsLR);

			if (vecMatchPointPairsLR.size() < 8)
				throw BaseException("Not enough matched points");

			//visualize matching results
			Mat matMatched = visualizeKeypointMatches(matImgL, matImgR, vecMatchPointPairsLR);
			imwrite(strDirPath + "/matched_" + int2str(i - 1) + "_" + int2str(i) + ".jpg", matMatched);

			//convert an array of point pairs to two array of points
			vector<Point2f> vecMatchedPtsL, vecMatchedPtsR;
			pointPairToPoints(vecMatchPointPairsLR, vecMatchedPtsL, vecMatchedPtsR);

			//find the fundamental matrix
			Mat matF = findFundamentalMat(vecMatchedPtsL, vecMatchedPtsR, CV_FM_8POINT);

			//refine the point matches
			vector<Point2f> vecReMatchedPtsL, vecReMatchedPtsR;
			correctMatches(matF, vecMatchedPtsL, vecMatchedPtsR, vecReMatchedPtsL, vecReMatchedPtsR);

			//find the essential matrix
			Mat matE = findEssentialMat(vecReMatchedPtsL, vecReMatchedPtsR, vecCameraParam[i].K);

			//finding R t using essential matrix, decompose the matrix by SVD and verify valid solution among the four solutions by triangulation
			//the triangulated point should have positive depth
			Matx33d matR;
			Vec3d vecR, vecT;
			int inliers = recoverPose(matE, vecReMatchedPtsL, vecReMatchedPtsR, vecCameraParam[i].K, matR, vecT);
			cout << matR << endl;
			Rodrigues(matR, vecR);

			//caculate current rotation and translation vector based on the reference frame
			composeRT(vecCameraParam[i - 1].R, vecCameraParam[i - 1].T, vecR, vecT, vecCameraParam[i].R, vecCameraParam[i].T);

			//write the camera pose
			fout << vecImgNames[i].substr(vecImgNames[i].find_last_of('\\')+1) << " ";
			vecCameraParam[i].writeCameraPose(fout);

			//print result
			cout << "Current RT based on reference:" << endl;
			cout << "R:" << vecCameraParam[i].R << endl;
			cout << "t:" << vecCameraParam[i].T << endl;

			////triangulate the points to verify the result;
			//Mat matPtsH, matPts;
			//triangulatePoints(vecCameraParam[i - 1].getProjectionMatrix(), vecCameraParam[i].getProjectionMatrix(), vecReMatchedPtsL, vecReMatchedPtsR, matPtsH);

			////mat to vector for input
			//vector<Vec4f> vecP;
			//mat2vec4d(matPtsH, vecP, false);

			////sparse point cloud file
			//ofstream foutPoint(strDirPath + "/points_"+int2str(i-1)+"_"+int2str(i)+".txt");
			//convertPointsFromHomogeneous(vecP, matPts);
			//writeMat(foutPoint, matPts);
			//foutPoint.close();

			int iDepthL = 3;
			int iDepthU = 100;

			Point ptTest(340, 190);

			circle(matImgRef, ptTest, 3, Scalar(255, 255, 0), 1, CV_AA);
			line(matImgR,
				projectPointWithDepth(vecCameraParam[0].K, ptTest, vecCameraParam[i].R, vecCameraParam[i].T, vecCameraParam[i].K, iDepthL),
				projectPointWithDepth(vecCameraParam[0].K, ptTest, vecCameraParam[i].R, vecCameraParam[i].T, vecCameraParam[i].K, iDepthU),
				Scalar(0, 255, 255), 1.3, CV_AA);
			showTwoImage("Ref", matImgRef, matImgR, true);
		}
		fout.close();
	}

	Mat matDepth;
	calcDepthMapWinnerTakesAll(vecImgNames, vecCameraParam, matDepth, 5);

	system("pause");
}