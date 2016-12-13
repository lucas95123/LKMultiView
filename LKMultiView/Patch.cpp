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

float Patch::calcNCC(Patch & p1, Patch & p2, NCC mode)
{
	if (mode == NCC::RGB)
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

		return fresult / 3.f;
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