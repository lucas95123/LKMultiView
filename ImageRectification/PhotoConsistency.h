#pragma once
#include <vector>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

class Patch
{
private:
	int m_patchSize;
	int m_channels;
public:
	Mat_<Vec3d> matIntensity;
	Mat_<Vec3d> average();
	double stdDeri();
	Patch(vector<Vec3d> intensity);
};

double calcNCC(Patch &p1, Patch &p2);