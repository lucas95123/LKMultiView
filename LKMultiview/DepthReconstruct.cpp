#include "DepthReconstruct.h"

/**@brief project points in the reference frame to other frame with specified depth
@param Kref intrinsic matrix of the reference camera
@param pt the point ro be projected
@param rvec rotation vector of the frame to be projected
@param tvec translation vector the the frame to be projected
@param K2 intrinsic matrix of the frame to be projected
@param depth the specified depth
*/
Point2d projectPointWithDepth(const Matx33d & Kref, Point pt, Vec3d rvec, Vec3d tvec, Matx33d K2, double depth)
{
	//to homogeneous point
	Vec3d vecPtHomo(pt.x, pt.y, 1);

	//to point in the camera coordinate, because the ref frame camera coordinate 
	//equals world coordinate so only multi the inv of intrinsic
	Vec3d vecPt3D = Kref.inv()*vecPtHomo;

	//multiply by depth to get the actual guessed 3d point coordinate
	vecPt3D *= depth;

	//pack into vector
	vector<Vec3d> vecPts3D;
	vecPts3D.push_back(vecPt3D);

	//project the point to the other frame
	Mat matPtImgR;
	projectPoints(vecPts3D, rvec, tvec, K2, noArray(), matPtImgR);

	return Point2d(round(matPtImgR.at<Vec2d>(0)[0]), round(matPtImgR.at<Vec2d>(0)[1]));
}

/**@calculate the index of the most likely depth
@param the Photo-Consistency Response matrix
*/
int calcDepthFromPCResponse(const Mat& matPC)
{
	Mat matAvg=Mat(1, matPC.cols,CV_64FC1);
	for (int col = 0; col < matPC.cols; col++)
	{
		double sum = 0.0;
		for (int row = 0; row < matPC.rows; row++)
			sum += matPC.at<double>(row, col);
		matAvg.at<double>(col) = sum / matPC.rows;
	}
	int iMax = 0;
	double dMax = 0.0;
	dMax = matAvg.at<double>(0);
	for (int i = 0; i < matAvg.cols; i++)
	{
		if (matAvg.at<double>(i) > dMax)
		{
			dMax = matAvg.at<double>(i);
			iMax = i;
		}
	}
	if (dMax < 0)
		return -1;
	return iMax;
}

/**@brief calculate the depth map with winner-takes-all strategy
@param vecImageName vector of the image filenames
@param vecCameraParam vector of the camera parameter
@param matDepth output matrix of depth
@param patchSize size of the patch
*/
void calcDepthMapWinnerTakesAll(const vector<string> &vecImgNames, const vector<CameraParameter> &vecCameraParam, Mat &matDepth, int patchSize)
{
	assert(vecCameraParam.size() == vecImgNames.size() && patchSize % 2 == 1);

	double dDepthL = 6;
	double dDepthU = 15;
	double dStep = 0.1;

	vector<Mat> vecImg;
	for (int i = 0; i < vecImgNames.size(); i++)
		vecImg.push_back(imread(vecImgNames[0]));

	Mat matImgRef = vecImg[0];
	matDepth = Mat::zeros(matImgRef.rows, matImgRef.cols, CV_64FC1);

	int iWidth = matImgRef.cols;
	int iHeight = matImgRef.rows;

	//for every pixel in the reference frame caculate depth
	for (int row = patchSize / 2; row < matImgRef.rows - patchSize / 2; row++)
	{
		for (int col = patchSize / 2; col < matImgRef.cols - patchSize / 2; col++)
		{
			//mat storing Photo-Consistency response of different depth in different frame column is the frame row is the response to different depth
			Mat matPCScore=Mat::zeros(vecCameraParam.size(), (dDepthU - dDepthL) / dStep, CV_64FC1);
			
			Point ptL(col, row);
			//for every possible depth in depth range
			for (int iImgIdx = 1; iImgIdx < vecCameraParam.size(); iImgIdx++)
			{
				Point prevPoint;
				for (double d = dDepthL; d < dDepthU; d += dStep)
				{
					Point ptR = projectPointWithDepth(vecCameraParam[0].K, ptL, vecCameraParam[iImgIdx].R, vecCameraParam[iImgIdx].T, vecCameraParam[iImgIdx].K, d);

					if (d != dDepthL)
						if (prevPoint.x<0 && ptR.x<prevPoint.x || 
							prevPoint.x>iWidth &&ptR.x>prevPoint.x || 
							prevPoint.y<0 && ptR.y<prevPoint.y || 
							prevPoint.y>iHeight &&ptR.y>prevPoint.y)
							break;
					prevPoint = ptR;

					double dPCScore;
					if (ptR.x >= patchSize / 2 && 
						ptR.x < matImgRef.cols - patchSize / 2 
						&& ptR.y >= patchSize / 2
						&& ptR.y < matImgRef.rows - patchSize / 2)
						dPCScore = calcNCC(getPatchFromImage(matImgRef, ptL, 3), getPatchFromImage(vecImg[iImgIdx], ptR, 3), NCC::RGB);
					else
						dPCScore = 0;
					matPCScore.at<double>(iImgIdx, (d - dDepthL) / dStep)=dPCScore;
				}
			}
			double dp = calcDepthFromPCResponse(matPCScore);;
			if (dp != -1)
				dp = dp*dStep + dDepthL;
			matDepth.at<double>(row, col) = dp;
		}
	}
	matDepth *= 100;
	showImage("depth", matDepth, 0);
	imwrite("depth.png", matDepth);
}