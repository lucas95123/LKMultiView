#include "Reconstruct.h"
#include "Util.h"

/**@brief project points in the reference frame to other frame with specified depth
@param Kref intrinsic matrix of the reference camera
@param pt the point ro be projected
@param rvec rotation vector of the frame to be projected
@param tvec translation vector the the frame to be projected
@param K2 intrinsic matrix of the frame to be projected
@param depth the specified depth
*/
void projectPointWithDepth(const Matx33d &K, Point2f &pt, const Mat &rmat, const Mat &tmat, float lower, float upper, float step, Mat &matPtR)
{
	int count = (upper - lower) / step;

	//convert to homogeneous point
	Mat matPt(count, 3, CV_32FC1);

	//to point in the camera coordinate, because the ref frame camera coordinate 
	//equals world coordinate so only multi the inv of intrinsic
	Vec3f vecPt3D = K.inv()*Vec3f(pt.x, pt.y, 1);

	//multiply by depth to get the actual guessed 3d point coordinate
	for (int i = 0; i < count; i++)
	{
		float depth = lower + step*i;
		matPt.at<float>(i, 0) = vecPt3D(0)*depth;
		matPt.at<float>(i, 1) = vecPt3D(1)*depth;
		matPt.at<float>(i, 2) = vecPt3D(2)*depth;
	}

	//convert from rotation matrix to vector
	Vec3d rvec;
	Rodrigues(rmat, rvec);

	//convert from translation matrix to vector
	Vec3d tvec(tmat.at<double>(0), tmat.at<double>(1), tmat.at<double>(2));

	//project the point to the other frame
	projectPoints(matPt, rvec, tvec, K, noArray(), matPtR);
}

/**@calculate the index of the most likely depth
@param the Photo-Consistency Response matrix
@param the lower depth bound
@param the upper depth bound
@param the depth step
*/
float calcDepthFromPCResponse(const Mat& matPC, Metric metric, float dL, float dU, float dS)
{
	//finding the average of PC Response
	Mat matAvg = Mat(1, matPC.cols, CV_32FC1);
	for (int col = 0; col < matPC.cols; col++)
	{
		float sum = 0.0;
		for (int row = 0; row < matPC.rows; row++)
			sum += matPC.at<float>(row, col);
		matAvg.at<float>(col) = sum / matPC.rows;
	}
	int iMax = 0;
	float dMax = 0.0;
	dMax = matAvg.at<float>(0);
	for (int i = 0; i < matAvg.cols; i++)
	{
		if (matAvg.at<float>(i) > dMax)
		{
			dMax = matAvg.at<float>(i);
			iMax = i;
		}
	}

	if (dMax < 0.55)
		return 0;
	return	dL + iMax*dS;
}

/**@calculate the index of the most likely depth using robust NCC
@param the Photo-Consistency Response matrix
@param the lower depth bound
@param the upper depth bound
@param the depth step
*/
float calcDepthFromPCResponseRobust(const Mat& matPC, Metric metric, float dL, float dU, float dS)
{
	//finding the average of PC Response
	Mat matAvg = Mat(1, matPC.cols, CV_32FC1);
	for (int col = 0; col < matPC.cols; col++)
	{
		float sum = 0.0;
		for (int row = 0; row < matPC.rows; row++)
			sum += matPC.at<float>(row, col);
		matAvg.at<float>(col) = sum / matPC.rows;
	}
	int iMax = 0;
	float dMax = 0.0;
	dMax = matAvg.at<float>(0);
	for (int i = 0; i < matAvg.cols; i++)
	{
		if (matAvg.at<float>(i) > dMax)
		{
			dMax = matAvg.at<float>(i);
			iMax = i;
		}
	}

	if (dMax < 0.55)
		return 0;
	return	dL + iMax*dS;
}

/**@calculate the index of the most likely depth using parzen window method
@param the Photo-Consistency Response matrix
@param the lower depth bound
@param the upper depth bound
@param the depth step
*/
float calcDepthFromPCResponseParzen(const Mat& matPC, Metric metric, float dL, float dU, float dS)
{
	//finding the average of PC Response
	Mat matAvg = Mat(1, matPC.cols, CV_32FC1);
	for (int col = 0; col < matPC.cols; col++)
	{
		float sum = 0.0;
		for (int row = 0; row < matPC.rows; row++)
			sum += matPC.at<float>(row, col);
		matAvg.at<float>(col) = sum / matPC.rows;
	}
	int iMax = 0;
	float dMax = 0.0;
	dMax = matAvg.at<float>(0);
	for (int i = 0; i < matAvg.cols; i++)
	{
		if (matAvg.at<float>(i) > dMax)
		{
			dMax = matAvg.at<float>(i);
			iMax = i;
		}
	}

	if (dMax < 0.55)
		return 0;
	return	dL + iMax*dS;
}

/**@brief calculate the depth map from multiple images and their relative position from reference image
@param matDepth the output depth map
@param vecImageNames the input list of image names
@param K the camera intrinsic matrix
@param vecRotation list of rotations obtained by trackCamera()
@param vecTranslation list of translations obtained by trackCamera()
@param method for Photo-Consistency metric, Reconstruct::NCC etc
@param patchSize size of the matching patch
@param lower min value of searching depth
@param upper max value of searching depth
@param step step of searching depth
*/
void calcDepthMapFromMultiView(
	Mat &matDepth,
	vector<string> &vecImgNames,
	Mat &K,
	vector<Mat> &vecRotation,
	vector<Mat> &vecTranslation,
	Reconstruct method,
	Metric metric,
	int patchSize,
	float lower,
	float upper,
	float step)
{
	vector<Mat> vecImg;
	for (int i = 0; i < vecImgNames.size(); i++)
		vecImg.push_back(imread(vecImgNames[i]));

	Mat matImgRef = vecImg[0];
	matDepth = Mat::zeros(matImgRef.rows, matImgRef.cols, CV_32FC1);

	int iWidth = matImgRef.cols;
	int iHeight = matImgRef.rows;
	int iSteps = (upper - lower) / step;

	Mat matPt;
	Patch patchL(patchSize);
	Patch patchR(patchSize);
 
	//Mat matL = vecImg[0].clone();
	//Mat matR = vecImg[1].clone();

	//Point2f ptTest = Point2f(485, 315);
	//projectPointWithDepth(
	//	K,
	//	ptTest,
	//	vecRotation[1],
	//	vecTranslation[1],
	//	lower ,
	//	upper,
	//	step,
	//	matPt
	//);

	//circle(matL, ptTest, 3, Scalar(255, 255, 0), 1, CV_AA);
	//for (int i = 0; i < matPt.rows; i++)
	//{
	//	circle(matR, Point(matPt.at<float>(i, 0), matPt.at<float>(i, 1)), 3, Scalar(255, 255, 0), 1, CV_AA);
	//	circle(matR, Point(matPt.at<float>(i, 0), matPt.at<float>(i, 1)), 3, Scalar(255, 255, 0), 1, CV_AA);
	//	if (matPt.at<float>(i, 0) >= patchSize / 2 && matPt.at<float>(i, 0) < iWidth - patchSize&&matPt.at<float>(i, 1) >= patchSize / 2 && matPt.at<float>(i, 1) <= iHeight - patchSize / 2)
	//	{
	//		getPatchFromImage(matImgRef, ptTest, patchL);
	//		getPatchFromImage(vecImg[1], Point2f(matPt.at<float>(i, 0), matPt.at<float>(i, 1)), patchR);
	//		//patchL.print();
	//		//patchL.visualize();
	//		//patchR.print();
	//		//patchR.visualize();
 //        	cout << Patch::calcNCC(patchL, patchR, metric) << endl;
	//		cout << lower + i*step << endl;
	//	}

	//	Mat matResult;
	//	hconcat(matL, matR, matResult);
	//	showImage("Result", matResult, 0);
	//	matResult.release();
	//}

	for (int row = patchSize / 2; row < iHeight - patchSize / 2; row++)
	{
		for (int col = patchSize / 2; col < iWidth - patchSize / 2; col++)
		{

			//mat storing Photo-Consistency response of different depth in different frame column is the frame row is the response to different depth
			Mat matPCScore = Mat::zeros(vecRotation.size()-1, iSteps, CV_32FC1);

			//current point in the reference image
			Point2f ptL(col, row);

			//point in the other image
			Point2f ptR;

			//for every possible depth in depth range
			for (int iImgIdx = 1; iImgIdx < vecRotation.size(); iImgIdx++)
			{
				//project 3d point to other image with every possible depth
				Mat matPtR;
				float dPCScore = 0.0;

				projectPointWithDepth(
					K,
					ptL,
					vecRotation[iImgIdx],
					vecTranslation[iImgIdx],
					lower,
					upper,
					step,
					matPtR
				);

				for (int i = 0; i < iSteps; i++)
				{
					ptR.x = matPtR.at<float>(i, 0);
					ptR.y = matPtR.at<float>(i, 1);
					if (ptR.x >= patchSize / 2 && ptR.x < iWidth - patchSize&&ptR.y >= patchSize / 2 && ptR.y < iHeight - patchSize / 2)
					{
						getPatchFromImage(matImgRef, ptL, patchL);
						getPatchFromImage(vecImg[iImgIdx], ptR, patchR);
						dPCScore = Patch::calcNCC(patchL, patchR, metric);
					}
					else
						dPCScore = 0;
					matPCScore.at<float>(iImgIdx - 1, i) = dPCScore;
				}	
			}
			float dRes = calcDepthFromPCResponse(matPCScore, metric, lower, upper, step);
			//if (row == ptTest.y && col == ptTest.x)
			//{
			//	visualizePCScores(matPCScore, 500);
			//	cout << " x: " << col << " y: " << row << " d: " << dRes << endl;
			//}
			matDepth.at<float>(row, col) = dRes;
		}
		cout << "Complete Percent: " << ((float)row) / iHeight * 100 << "%              " << "\r";
	}
}
