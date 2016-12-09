#include <iostream>
#include <fstream>

#include "Exceptions.h"
#include "CameraParameter.h"
#include "Util.h"
#include "CvUtil.h"
#include "KeyPointMatcher.h"
#include "Rectification.h"

#include <opencv2\opencv.hpp>
#include <opencv2\viz.hpp>

#define M_PI 3.1415926536

using namespace std;
using namespace cv;
using namespace viz;

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

		Vec3f vecEuler=rotationMatrixToEulerAngles((Mat)matR);

		cout << "Euler Angle: " << vecEuler / M_PI * 180 << endl;

		//caculate current rotation and translation vector based on the reference frame
		composeRT(vecCameraParam[i-1].R, vecCameraParam[i-1].T, vecR, vecT, vecCameraParam[i].R, vecCameraParam[i].T);

		//write the camera pose
		vecCameraParam[i].writeCameraPose(fout);

		//print result
		cout << "Previous RT:" << endl;
		cout << vecCameraParam[i - 1].R << endl;
		cout << vecCameraParam[i - 1].T <<	endl;

		cout << "Current RT:" << endl;
		cout << vecR << endl;
		cout << vecT <<	endl;

		cout << "Current RT based on reference:" << endl;
		cout << vecCameraParam[i].R << endl;
		cout << vecCameraParam[i].T << endl;

		cout << "projection matrix" << endl;
		cout << vecCameraParam[i - 1].getProjectionMatrix() << endl;

		//Mat matPts4D;
		//triangulatePoints(vecCameraParam[i - 1].getProjectionMatrix(), vecCameraParam[i].getProjectionMatrix(), vecReMatchedPtsL, vecReMatchedPtsR, matPts4D);

		//vector<Vec4f> vecP;
		//for (int i = 0; i < vecReMatchedPtsL.size(); i++)
		//{

		//	Vec4f P(matPts4D.at<float>(0, i),
		//		matPts4D.at<float>(1, i),
		//		matPts4D.at<float>(2, i),
		//		matPts4D.at<float>(3, i));
		//	vecP.push_back(P);
		//}

		//Mat matPts;
		////sparse point cloud file
		//ofstream foutPoint(strDirPath + "/points_"+int2str(i-1)+"_"+int2str(i)+".txt");
		//convertPointsFromHomogeneous(vecP, matPts);
		//writeMat(foutPoint, matPts);
		//fout.close();
	}
	fout.close();
	int iDepthUpper = 100;
	int iDepthLower = 1;
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
					vector<Point3d> vecP;
					Mat matVecp;
					vecP.push_back(Point3d(row, col, d));
					projectPoints(vecP, vecCameraParam[iImgIndex].R, vecCameraParam[iImgIndex].T, vecCameraParam[iImgIndex].K, noArray(), matVecp);
					cout << matVecp << endl;
				}
			}
		}
	}

}