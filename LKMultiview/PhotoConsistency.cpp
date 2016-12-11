#include "PhotoConsistency.h"

Mat Patch::average()
{
	return Mat();
}

Mat Patch::average(RGB channel)
{
	Mat matChannel = data(channel);

	double res = mean(matChannel)[0];

	for (int i = 0; i < m_patchSize; i++)
		matChannel.at<double>(i) = res;

	return matChannel;
}

Mat Patch::data(RGB channel)
{
	int iChannel;
	if (channel == RGB::R)
		iChannel = 2;
	else if (channel == RGB::G)
		iChannel = 1;
	else
		iChannel = 0;
	
	Mat matResult;
	matResult = Mat(m_patchSize, 1, CV_64FC1);
	for (int i = 0; i < m_patchSize; i++)
		matResult.at<double>(i) = (double)matPatch.at<uchar>(m_patchSize * iChannel + i);

	return matResult;
}

double Patch::stdDeri()
{
	return 0.0;
}

double Patch::stdDeri(RGB channel)
{
	Mat matChannel = data(channel);
	Mat matMean, matStd;
	meanStdDev(matChannel, matMean, matStd);
	return matStd.at<double>(0, 0);
}

Patch::Patch(Mat patch)
{
	assert(patch.rows % 3 == 0);
	m_patchSize = patch.rows / 3;
	matPatch = patch;
}

void Patch::print()
{
	cout << matPatch << endl;
}

double calcNCC(Patch & p1, Patch & p2, NCC mode)
{
	if (mode == NCC::RGB)
	{
		double stderi;
		//calculate NCC for R channel					 											
		double dRChannel = (p1.data(RGB::R) - p1.average(RGB::R)).dot((p2.data(RGB::R) - p2.average(RGB::R))) / p1.size();
		stderi = p1.stdDeri(RGB::R)*p2.stdDeri(RGB::R);
		dRChannel = stderi == 0 ? 0 : dRChannel / stderi;
		//calculate NCC for G channel					 											
		double dGChannel = (p1.data(RGB::G) - p1.average(RGB::G)).dot((p2.data(RGB::G) - p2.average(RGB::G))) / p1.size();
		stderi = p1.stdDeri(RGB::G)*p2.stdDeri(RGB::G);
		dGChannel = stderi == 0 ? 0 : dGChannel / stderi;
		//calculate NCC for B channel					 											
		double dBChannel = (p1.data(RGB::B) - p1.average(RGB::B)).dot((p2.data(RGB::B) - p2.average(RGB::B))) / p1.size();
		stderi = p1.stdDeri(RGB::B)*p2.stdDeri(RGB::B);
		dBChannel = stderi == 0 ? 0 : dBChannel / stderi;

		return (dRChannel + dGChannel + dBChannel) / 3.0;
	}
	return 0.0;
}

Patch getPatchFromImage(const Mat & matImg, Point pt, int size)
{
	assert(size % 2 != 0);
	Mat matPatch;
	matPatch = Mat(size*size*3, 1, CV_8UC1);
	
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
		{
			try
			{
				matPatch.at<uchar>(i*size + j) = matImg.at<Vec3b>(Point(pt.x - size / 2 + j, pt.y - size / 2 + i))[0];
			}
			catch (cv::Exception)
			{
				matPatch.at<uchar>(i*size + j) = 0;
			}
			try
			{
				matPatch.at<uchar>(size*size + i*size + j) = matImg.at<Vec3b>(Point(pt.x - size / 2 + j, pt.y - size / 2 + i))[1];
			}
			catch (cv::Exception)
			{
				matPatch.at<uchar>(size*size + i*size + j) = 0;
			}
			try
			{
				matPatch.at<uchar>(2 * size*size + i*size + j) = matImg.at<Vec3b>(Point(pt.x - size / 2 + j, pt.y - size / 2 + i))[2];
			}
			catch (cv::Exception)
			{
				matPatch.at<uchar>(2 * size*size + i*size + j) = 0;
			}
		}

	return Patch(matPatch);
}
