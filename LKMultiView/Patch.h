#pragma once
#include <vector>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

enum class NCC {
	RGB, SINGLE, MIXED
};

enum class RGB
{
	R, G, B
};

class Patch
{
private:
	int m_patchSize;
	int m_channels;
public:
	Patch(int size) {
		m_patchSize = size*size;
	}
	~Patch()
	{
		delete[]data;
	}
	float *data;
	float average(RGB channel);
	float stdDeri(RGB channel);
	float *getPtr(RGB channel);
	void setSize(int size)
	{
		m_patchSize = size;
	}
	int getSize() {
		return m_patchSize;
	}
	void print()
	{
		cout << "R: ";
		for (int i = 0; i < m_patchSize; i++)
		{
			cout << data[i] << " ";
		}
		cout << endl;
		cout << "G: ";
		for (int i = m_patchSize; i < m_patchSize*2; i++)
		{
			cout << data[i] << " ";
		}
		cout << endl;
		cout << "B: ";
		for (int i = m_patchSize*2; i < m_patchSize*3; i++)
		{
			cout << data[i] << " ";
		}
		cout << endl;
	}
	void visualize()
	{
		int size = sqrt(m_patchSize);
		Mat matVis(size, size, CV_8UC3);
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				matVis.at<Vec3b>(i, j)[0] = data[i*size + j];
				matVis.at<Vec3b>(i, j)[1] = data[i*size + j + m_patchSize];
				matVis.at<Vec3b>(i, j)[1] = data[i*size + j + m_patchSize * 2];
			}
		}
		namedWindow("visualize patch", WINDOW_KEEPRATIO);
		imshow("visualize patch", matVis);
		waitKey(0);
		destroyWindow("visualize patch");
	}
	static float calcNCC(Patch & p1, Patch & p2, NCC mode);
};

void getPatchFromImage(const Mat & matImg, Point2f &pt, Patch &patch);
