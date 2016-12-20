#include <GCoptimization.h>

#include "Reconstruct.h"
#include "Util.h"

#define LABEL_UNKNOWN 0
#define TAU1 60
#define TAU2 150

/**@brief project points in the reference frame to other frame with specified depth
@param Kref intrinsic matrix of the reference camera
@param pt the point ro be projected
@param rvec rotation vector of the frame to be projected
@param tvec translation vector the the frame to be projected
@param K2 intrinsic matrix of the frame to be projected
@param depth the specified depth
*/
void projectPointWithDepth(
	const Matx33d &K,
	const Mat&rmatRef,
	const Mat&tmatRef,
	Point2f &pt,
	const Mat &rmat,
	const Mat &tmat,
	float lower,
	float upper,
	float step,
	Mat &matPtR)
{
	int count = (upper - lower) / step;

	//convert to homogeneous point
	Mat matPt(count, 3, CV_32FC1);

	//to point in the camera coordinate, because the ref frame camera coordinate 
	//equals world coordinate so only multi the inv of intrinsic
	Vec3f vecPt3D = K.inv()*Vec3f(pt.x, pt.y, 1);

	//projection matrix's inverse from rotation and translation
	Mat projRefInv = Mat::zeros(4, 4, CV_32FC1);
	Mat tmpRotation = rmatRef.t();
	Mat tmpTranslation = -rmatRef.t()*tmatRef;
	tmpRotation.convertTo(projRefInv(Range(0, 3), Range(0, 3)), CV_32FC1);
	tmpTranslation.convertTo(projRefInv(Range(0, 3), Range(3, 4)), CV_32FC1);
	projRefInv.at<float>(3, 3) = 1.f;

	//multiply by depth to get the actual guessed 3d point coordinate
	Vec4f vecCoord;
	Mat res(4, 1, CV_32FC1);
	for (int i = 0; i < count; i++)
	{
		float depth = lower + step*i;
		vecCoord(0) = vecPt3D(0)*depth;
		vecCoord(1) = vecPt3D(1)*depth;
		vecCoord(2) = vecPt3D(2)*depth;
		vecCoord(3) = 1.f;
		res = projRefInv*Mat(vecCoord);
		matPt.at<float>(i, 0) = res.at<float>(0);
		matPt.at<float>(i, 1) = res.at<float>(1);
		matPt.at<float>(i, 2) = res.at<float>(2);
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
void calcDepthFromPCResponseWithMRF(
	const Mat& matPC,
	GCoptimizationGridGraph *gc,
	int pixelIndex,
	Metric metric,
	float dL,
	float dU,
	float dS,
	float threshold,
	bool visualize = false)
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
	for (int i = 1; i < matAvg.cols + 1; i++)
		gc->setDataCost(pixelIndex, i, (int)(100 - matAvg.at<float>(i) * 100));
	gc->setDataCost(pixelIndex, LABEL_UNKNOWN, TAU1);
}

/**@calculate the index of the most likely depth
@param the Photo-Consistency Response matrix
@param the lower depth bound
@param the upper depth bound
@param the depth step
*/
float calcDepthFromPCResponse(
	const Mat& matPC,
	Metric metric,
	float dL,
	float dU,
	float dS,
	float threshold,
	bool visualize = false)
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

	if (visualize)
		visualizePCScores(matPC, matAvg, 500);

	if (dMax < threshold)
		return 0;
	return	dL + iMax*dS;
}

/**@calculate the index of the most likely depth using parzen window method
@param the Photo-Consistency Response matrix
@param the lower depth bound
@param the upper depth bound
@param the depth step
*/
void calcDepthFromPCResponseParzenWithMRF(
	const Mat& matPC,
	GCoptimizationGridGraph *gc,
	int pixelIndex,
	Metric metric,
	float dL,
	float dU,
	float dS,
	int kernelSize,
	bool visualize)
{
	//finds local maximas
	Mat matLocalMaxima = Mat::zeros(matPC.rows, matPC.cols, CV_32FC1);
	for (int i = 0; i < matPC.rows; i++)
	{
		for (int j = 1; j < matPC.cols - 1; j++)
		{
			if (matPC.at<float>(i, j) > matPC.at<float>(i, j - 1) && matPC.at<float>(i, j) > matPC.at<float>(i, j + 1))
				matLocalMaxima.at<float>(i, j) = matPC.at<float>(i, j);
		}
	}

	Mat matResult = Mat::zeros(1, matPC.cols, CV_32FC1);
	//using parzen window
	GaussianKernel kernel = GaussianKernel(3);
	for (int i = 0; i < matLocalMaxima.cols; i++)
	{
		float sum = 0.f;
		int count = 0.f;
		for (int j = i - kernelSize / 2; j < i + kernelSize / 2; j++)
		{
			if (j < 0 || j >= matLocalMaxima.cols)
			{
				sum += 0;
				continue;
			}
			for (int k = 0; k < matLocalMaxima.rows; k++)
			{
				float tmp = matLocalMaxima.at<float>(k, j);
				if (tmp == 0.f)
					continue;
				else
				{
					sum += tmp*kernel.getVal(j - i);
					count++;
				}
			}
		}
		matResult.at<float>(i) = sum;
		gc->setDataCost(pixelIndex, i, min(100, (int)(100 - sum * 100)));
	}
	gc->setDataCost(pixelIndex, LABEL_UNKNOWN, 600);
}

/**@calculate the index of the most likely depth using parzen window method
@param the Photo-Consistency Response matrix
@param the lower depth bound
@param the upper depth bound
@param the depth step
*/
float calcDepthFromPCResponseParzen(
	const Mat& matPC,
	Metric metric,
	float dL,
	float dU,
	float dS,
	float threshold,
	int kernelSize,
	bool visualize)
{
	//finds local maximas
	Mat matLocalMaxima = Mat::zeros(matPC.rows, matPC.cols, CV_32FC1);
	for (int i = 0; i < matPC.rows; i++)
	{
		for (int j = 1; j < matPC.cols - 1; j++)
		{
			if (matPC.at<float>(i, j) > matPC.at<float>(i, j - 1) && matPC.at<float>(i, j) > matPC.at<float>(i, j + 1))
				matLocalMaxima.at<float>(i, j) = matPC.at<float>(i, j);
		}
	}

	Mat matResult = Mat::zeros(1, matPC.cols, CV_32FC1);
	//using parzen window
	GaussianKernel kernel = GaussianKernel(3);
	for (int i = kernelSize / 2; i < matLocalMaxima.cols - kernelSize / 2; i++)
	{
		float sum = 0.f;
		int count = 0.f;
		for (int j = i - kernelSize / 2; j < i + kernelSize / 2; j++)
		{
			for (int k = 0; k < matLocalMaxima.rows; k++)
			{
				float tmp = matLocalMaxima.at<float>(k, j);
				if (tmp == 0.f)
					continue;
				else
				{
					sum += tmp*kernel.getVal(j - i);
					count++;
				}
			}
		}
		matResult.at<float>(0, i) = sum;
	}

	//finding global maxima of the averaged curve
	int iMax = 0;
	float dMax = 0.0;
	dMax = matResult.at<float>(0);
	for (int i = 0; i < matResult.cols; i++)
	{
		if (matResult.at<float>(i) > dMax)
		{
			dMax = matResult.at<float>(i);
			iMax = i;
		}
	}

	if (dMax < threshold)
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
void calcDepthMapFromMultiViewWithMRF(
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
	//vector storing images
	vector<Mat> vecImg;
	for (int i = 0; i < vecImgNames.size(); i++)
		vecImg.push_back(imread(vecImgNames[i]));

	//set the first image as the reference image
	Mat matImgRef = vecImg[0];

	//initialize the depth map
	matDepth = Mat::zeros(matImgRef.rows, matImgRef.cols, CV_32FC1);

	int iWidth = matImgRef.cols;
	int iHeight = matImgRef.rows;
	int iSteps = (upper - lower) / step;

	//sliding window patches
	Patch patchL(patchSize);
	Patch patchR(patchSize);

	//GCoptimization, one more state for unknown label
	GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(iWidth, iHeight, iSteps + 1);

	for (int row = 0; row < iHeight; row++)
	{
		for (int col = 0; col < iWidth; col++)
		{
			//for borders force to be unknown label penalize regular label
			if (row < patchSize / 2 || row >= iHeight - patchSize / 2 || col < patchSize / 2 || col >= iWidth - patchSize / 2)
			{
				for (int l = 1; l < iSteps + 1; l++)
					gc->setDataCost(row*iWidth + col, l, 100);
				gc->setDataCost(row*iWidth + col, LABEL_UNKNOWN, 0);
				continue;
			}
			//mat storing Photo-Consistency response of different depth in different frame column is the frame row is the response to different depth
			Mat matPCScore = Mat::zeros(vecRotation.size() - 1, iSteps, CV_32FC1);

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
					vecRotation[0],
					vecTranslation[0],
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
					//get patches from reference image and other image
					ptR.x = matPtR.at<float>(i, 0);
					ptR.y = matPtR.at<float>(i, 1);
					if (ptR.x >= patchSize / 2 && ptR.x < iWidth - patchSize&&ptR.y >= patchSize / 2 && ptR.y < iHeight - patchSize / 2)
					{
						getPatchFromImage(matImgRef, ptL, patchL);
						getPatchFromImage(vecImg[iImgIdx], ptR, patchR);
						//calculate photo-consistency metric
						dPCScore = Patch::calcNCC(patchL, patchR, metric);
					}
					else
						dPCScore = 0;
					matPCScore.at<float>(iImgIdx - 1, i) = dPCScore;
				}
			}
			calcDepthFromPCResponseWithMRF(
				matPCScore,
				gc,
				row*iWidth + col,
				metric,
				lower,
				upper,
				step,
				3,
				false
			);
		}
		cout << "Complete Percent: " << ((float)row) / iHeight * 100 << "%              " << "\r";
	}

	for (int l1 = 0; l1 < iSteps + 1; l1++)
		for (int l2 = 0; l2 < iSteps + 1; l2++)
		{
			if (l1 == LABEL_UNKNOWN && l2 == LABEL_UNKNOWN)
				gc->setSmoothCost(l1, l2, 0);
			else if (l1 == LABEL_UNKNOWN && l2 != LABEL_UNKNOWN || l1 != LABEL_UNKNOWN && l2 == LABEL_UNKNOWN)
				gc->setSmoothCost(l1, l2, TAU2);
			else
				gc->setSmoothCost(l1, l2, abs(l1 - l2));
		}

	cout << "Before optimization: " << gc->compute_energy() << endl;
	gc->expansion(10);
	cout << "After optimization: " << gc->compute_energy() << endl;

	for (int i = 0; i < iHeight*iWidth; i++)
		matDepth.at<float>(i) = gc->whatLabel(i) == 0 ? 0 : (gc->whatLabel(i)-1)*step + lower;

	delete gc;
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

	//int testIndex = 2;
	//
	//Mat matL = vecImg[0].clone();
	//Mat matR = vecImg[testIndex].clone();

	//Point2f ptTest = Point2f(200, 229);
	//projectPointWithDepth(
	//	K,
	//	vecRotation[0],
	//	vecTranslation[0],
	//	ptTest,
	//	vecRotation[testIndex],
	//	vecTranslation[testIndex],
	//	lower ,
	//	upper,
	//	step,
	//	matPt
	//);

	//circle(matL, ptTest, 3, Scalar(255, 255, 0), 1, CV_AA);
	//for (int i = 0; i < matPt.rows; i++)
	//{
	//	circle(matR, Point(matPt.at<float>(i, 0), matPt.at<float>(i, 1)), 3, Scalar(255, 255, 0), 1, CV_AA);                                                        
	//	if (matPt.at<float>(i, 0) >= patchSize / 2 && matPt.at<float>(i, 0) < iWidth - patchSize&&matPt.at<float>(i, 1) >= patchSize / 2 && matPt.at<float>(i, 1) <= iHeight - patchSize / 2)
	//	{
	//		getPatchFromImage(matImgRef, ptTest, patchL);
	//		getPatchFromImage(vecImg[testIndex], Point2f(matPt.at<float>(i, 0), matPt.at<float>(i, 1)), patchR);
	//		//patchL.print();
	//		Mat matPatch1=patchL.visualize();
	//		//patchR.print();
	//		Mat matPatch2=patchR.visualize();
	//     	cout << Patch::calcNCC(patchL, patchR, metric) << endl;
	//		cout << lower + i*step << endl;
	//		Mat matResult;
	//		hconcat(matPatch1, matPatch2, matResult);
	//		showImage("result", matResult, 0, WINDOW_KEEPRATIO);
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
			Mat matPCScore = Mat::zeros(vecRotation.size() - 1, iSteps, CV_32FC1);

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
					vecRotation[0],
					vecTranslation[0],
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
			float dRes = calcDepthFromPCResponse(matPCScore, metric, lower, upper, step, 0.5, false);
			//cout << dRes << endl;
			//if (row == ptTest.y && col == ptTest.x)
			//{
			//	float dRes = calcDepthFromPCResponseParzen(matPCScore, metric, lower, upper, step, 0.7, 3, true);
			//	cout << " x: " << col << " y: " << row << " d: " << dRes << endl;
			//	dRes = calcDepthFromPCResponseRobust(matPCScore, metric, lower, upper, step, 0.6, true);
			//	cout << " x: " << col << " y: " << row << " d: " << dRes << endl;
			//}
			matDepth.at<float>(row, col) = dRes;
		}
		cout << "Complete Percent: " << ((float)row) / iHeight * 100 << "%              " << "\r";
	}
}