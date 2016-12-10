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

	vector<string> vecImgNames = getListFiles(strDirPath, "png", false);
	vector<CameraParameter> vecCameraParam;

	try {
		readCameraParameters(vecCameraParam, strDirPath+"/temple.txt");	
	}
	catch (BaseException e)
	{
		cout<<e.what()<<endl;
		return 0;
	}

	//read the reference image and set it as the reference image
	Mat matImgRef=imread(vecImgNames[0]);
	vecCameraParam[0].setAsReference();

	//campos file
	ofstream fout(strDirPath + "/campos.txt");

	//caculate RT between every frame and the reference frame
	for (int i = 1; i < vecImgNames.size() && i<iRectNum; i++)
	{
		//read the current image
		Mat matImgL, matImgR;
		matImgL = imread(vecImgNames[i - 1]);
		matImgR = imread(vecImgNames[i]);

		//find the correspondence with SIFT descriptor and RANSAC the result
		vector<pair<Point2f, Point2f>> vecMatchPointPairsLR;
		getKeypointMatchesWithAllAlgorithms(matImgL, matImgR, vecMatchPointPairsLR);

		//visualize matching results
		Mat matMatched = visualizeKeypointMatches(matImgL, matImgR, vecMatchPointPairsLR);
		imwrite(strDirPath + "/matched_"+int2str(i-1)+"_"+int2str(i)+".jpg", matMatched);

		//convert an array of point pairs to two array of points
		vector<Point2f> vecMatchedPtsL, vecMatchedPtsR;
		pointPairToPoints(vecMatchPointPairsLR, vecMatchedPtsL, vecMatchedPtsR);

		//find the fundamental matrix
		Mat matF = findFundamentalMat(vecMatchedPtsL, vecMatchedPtsR, CV_FM_8POINT);

		//refine the point matches
		vector<Point2f> vecReMatchedPtsL, vecReMatchedPtsR;
		correctMatches(matF, vecMatchedPtsL, vecMatchedPtsR, vecReMatchedPtsL, vecReMatchedPtsR);

		//find the essential matrix
		Mat matE=findEssentialMat(vecReMatchedPtsL, vecReMatchedPtsR, vecCameraParam[i].K);
		
		//finding R t using essential matrix, decompose the matrix by SVD and verify valid solution among the four solutions by triangulation
		//the triangulated point should have positive depth
		Matx33d matR;
		Vec3d vecR, vecT;
		int inliers = recoverPose(matE, vecReMatchedPtsL, vecReMatchedPtsR, vecCameraParam[i].K, matR, vecT);
		cout << matR << endl;
		Rodrigues(matR, vecR);

		//caculate current rotation and translation vector based on the reference frame
		composeRT(vecCameraParam[i-1].R, vecCameraParam[i-1].T, vecR, vecT, vecCameraParam[i].R, vecCameraParam[i].T);

		//write the camera pose
		vecCameraParam[i].writeCameraPose(fout);

		//print result
		cout << "Current RT based on reference:" << endl;
		cout << "R:" << vecCameraParam[i].R << endl;
		cout << "t:" << vecCameraParam[i].T << endl;
		
		//triangulate the points to verify the result;
		Mat matPtsH, matPts;
		triangulatePoints(vecCameraParam[i - 1].getProjectionMatrix(), vecCameraParam[i].getProjectionMatrix(), vecReMatchedPtsL, vecReMatchedPtsR, matPtsH);

		//mat to vector for input
		vector<Vec4f> vecP;
		mat2vec4d(matPtsH, vecP, false);

		//sparse point cloud file
		ofstream foutPoint(strDirPath + "/points_"+int2str(i-1)+"_"+int2str(i)+".txt");
		convertPointsFromHomogeneous(vecP, matPts);
		writeMat(foutPoint, matPts);
		foutPoint.close();
	}
	fout.close();

	int iDepthLower = 0.0;
	int iDepthUpper = iDepthLower + 100;
	//for every pixel in the reference frame caculate depth
	for (int row = 0; row < matImgRef.rows; row++)
	{
		for (int col = 0; col < matImgRef.cols; col++)
		{
			//for every possible depth in depth range
			for (int d = iDepthLower; d < iDepthUpper; d++)
			{
				for (int iImgIndex = 1; iImgIndex < vecCameraParam.size(); iImgIndex++)
				{
					vector<Point2d> vecP(1;
					Mat matVecp;
					projectPoints(vecP, vecCameraParam[iImgIndex].R, vecCameraParam[iImgIndex].T, vecCameraParam[iImgIndex].K, noArray(), matVecp);
					cout << matVecp << endl;
				}
			}
		}
	}
	system("pause");
}