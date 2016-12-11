#pragma once
#include <vector>
#include <opencv2\opencv.hpp>

#include "CvUtil.h"

using namespace std;
using namespace cv;

enum class NCC{
	RGB, SINGLE, MIXED
};

class Patch
{
private:
	int m_patchSize;
	int m_channels;
public:
	Mat matPatch;
	Mat average();
	Mat average(RGB channel);
	Mat data(RGB channel);
	double stdDeri();
	double stdDeri(RGB channel);
	Patch(Mat patch);
	int size() {
		return m_patchSize;
	}
	void print();
};

double calcNCC(Patch &p1, Patch &p2, NCC mode);

Patch getPatchFromImage(const Mat &matImg, Point pt, int size);