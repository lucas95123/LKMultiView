#include <iostream>
#include <io.h>

#include <opencv2\opencv.hpp>

#include "Tracking.h"
#include "Util.h"
#include "Reconstruct.h"
#include "Patch.h"

using namespace std;

int main(int argc, char ** argv)
{
	string strDirPath = argv[1];
	int iPatchSize = str2int(argv[2]);
	float fLower = str2float(argv[3]);
	float fUpper = str2float(argv[4]);
	float fStep = str2float(argv[5]);

	vector<string> vecImgNames = getListFiles(strDirPath, "png", true);

	Mat K;
	Mat matDepth;
	vector<Mat> vecRotations;
	vector<Mat> vecTranslations;

	if (!readCameraIntrinsics(strDirPath, K))
	{
		cout << "Cannot read camera intrinsics" << endl;
		exit(0);
	}

	if(!readCameraPoses(strDirPath, vecRotations, vecTranslations))
		trackCamera(
			vecImgNames,
			K,
			vecRotations,
			vecTranslations,
			true,
			strDirPath);

	if (!readDepthMap(strDirPath, matDepth))
	{
		try {
			calcDepthMapFromMultiView(
				matDepth,
				vecImgNames,
				K,
				vecRotations,
				vecTranslations,
				Reconstruct::WINNER_TAKES_ALL,
				Metric::NCCRGB,
				iPatchSize,
				fLower,
				fUpper,
				fStep);
		}
		catch (Exception ex)
		{
			ex.what();
		}

		FileStorage fs(strDirPath + ".\\depth.xml", FileStorage::WRITE);
		fs << "depth" << matDepth;
		fs.release();
	}

	Mat matGray;
	normalizeAndRemapToGrayScale(matDepth, matGray);

	showImage("Depth", matGray, 0);
	imwrite(strDirPath + ".\\depth.jpg", matGray);

	system("pause");
}