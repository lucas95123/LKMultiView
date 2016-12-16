#include "Patch.h"


float * Patch::getPtr(RGB channel)
{
	int base = 0;
	switch (channel)
	{
	case RGB::B:return data;
	case RGB::G:return data + m_patchSize;
	case RGB::R:return data + m_patchSize * 2;
	}
	return nullptr;
}

float Patch::average(RGB channel)
{
	float fresult = 0.0;
	float *fptr = getPtr(channel);
	for (int i = 0; i < m_patchSize; i++)
		fresult += fptr[i];
	return fresult / m_patchSize;
}

float Patch::stdDeri(RGB channel)
{
	float fmean = average(channel);
	float *fptr = getPtr(channel);
	float fresult = 0.f;
	for (int i = 0; i < m_patchSize; i++)
	{
		fresult += pow(fptr[i] - fmean, 2);
	}
	return sqrt(fresult / m_patchSize);
}

float Patch::calcNCC(Patch & p1, Patch & p2, Metric metric)
{
	if (metric == Metric::NCCRGB)
	{
		//calculatie NCC for B channel
		float *fptrData1 = p1.getPtr(RGB::B);
		float *fptrData2 = p2.getPtr(RGB::B);
		float favg1 = p1.average(RGB::B);
		float favg2 = p2.average(RGB::B);
		float fresult = 0.f;
		float fresulttmp = 0.f;
		for (int i = 0; i < p1.m_patchSize; i++)
		{
			fresulttmp += (fptrData1[i] - favg1)*(fptrData2[i] - favg2);
		}
		fresult += fresulttmp / (p1.m_patchSize*p1.stdDeri(RGB::B)*p2.stdDeri(RGB::B));

		//calculatie NCC for G channel
		fptrData1 = fptrData1 + p1.m_patchSize;
		fptrData2 = fptrData2 + p2.m_patchSize;
		favg1 = p1.average(RGB::G);
		favg2 = p2.average(RGB::G);
		fresulttmp = 0.f;
		for (int i = 0; i < p1.m_patchSize; i++)
		{
			fresulttmp += (fptrData1[i] - favg1)*(fptrData2[i] - favg2);
		}
		fresult += fresulttmp / (p1.m_patchSize*p1.stdDeri(RGB::G)*p2.stdDeri(RGB::G));

		//calculatie NCC for R channel
		fptrData1 = fptrData1 + p1.m_patchSize;
		fptrData2 = fptrData2 + p2.m_patchSize;
		favg1 = p1.average(RGB::R);
		favg2 = p2.average(RGB::R);
		fresulttmp = 0.f;
		for (int i = 0; i < p1.m_patchSize; i++)
		{
			fresulttmp += (fptrData1[i] - favg1)*(fptrData2[i] - favg2);
		}
		fresult += fresulttmp / (p1.m_patchSize*p1.stdDeri(RGB::R)*p2.stdDeri(RGB::R));

        if (isnan<float>(fresult))
			return 0;
		return fresult / 3.f;
	}
	else if (metric == Metric::NCCSINGLE)
	{

	}
	else if (metric == Metric::NSSD)
	{
		int size = p1.getSize() * 3;
		float *fPtrData1 = p1.data;
		float *fPtrData2 = p2.data;
		float fmean1 = meanf(fPtrData1, size);
		float fmean2 = meanf(fPtrData2, size);
		float fstdev1 = stdevf(fPtrData1, size);
		float fstdev2 = stdevf(fPtrData2, size);
		float fsum = 0.f;
		for (int i = 0; i < size; i++)
			fsum += pow((fPtrData1[i] - fmean1) / fstdev1 - (fPtrData2[i] - fmean2) / fstdev2, 2);
		return exp(-fsum/25);
	}
	else if (metric == Metric::SAD)
	{
		int size = p1.getSize() * 3;
		float *fPtrData1 = p1.data;
		float *fPtrData2 = p2.data;
		float *fPtrDiff = new float[size];

		for (int i = 0; i < size; i++)
			fPtrDiff[i] = abs(fPtrData1[i]-fPtrData2[i]);

		float fresult=exp(-sumf(fPtrDiff, size) / stdevf(fPtrDiff, size)/196);
		delete[] fPtrDiff;
		return fresult;
	}
	return 0.0;
}

void getPatchFromImage(const Mat & matImg, Point2f &pt, Patch &patch)
{
	int size = sqrt(patch.getSize());
	assert(size % 2 != 0 && pt.x >= size / 2 && pt.y >= size / 2);
	if (patch.data == nullptr)
		patch.data = new float[size*size * 3];
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
		{
			patch.data[i*size + j] = matImg.at<Vec3b>(Point(pt.x - size / 2 + j, pt.y - size / 2 + i))[0];
			patch.data[size*size + i*size + j] = matImg.at<Vec3b>(Point(pt.x - size / 2 + j, pt.y - size / 2 + i))[1];
			patch.data[2 * size*size + i*size + j] = matImg.at<Vec3b>(Point(pt.x - size / 2 + j, pt.y - size / 2 + i))[2];
		}
}