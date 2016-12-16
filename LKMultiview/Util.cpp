#include "Util.h"
#include <time.h>

int str2int(string str)
{
	stringstream ss;
	ss << str;
	int res;
	ss >> res;
	return res;
}

float str2float(string str)
{
	stringstream ss;
	ss << str;
	float res;
	ss >> res;
	return res;
}

double str2double(string str)
{
	stringstream ss;
	ss << str;
	double res;
	ss >> res;
	return res;
}

string int2str(int i)
{
	stringstream ss;
	ss << i;
	string res;
	ss >> res;
	return res;
}

string float2str(float f)
{
	stringstream ss;
	ss << f;
	string res;
	ss >> res;
	return res;
}

string double2str(double d)
{
	stringstream ss;
	ss << d;
	string res;
	ss >> res;
	return res;
}

bool readCameraPoses(string path, vector<Mat> &vecRotations, vector<Mat> &vecTranslations)
{
	FileStorage fs(path + ".\\structure.yml", FileStorage::READ);
	fs["Rotations"] >> vecRotations;
	fs["Motions"] >> vecTranslations;
	if (vecRotations.size() > 0 && vecTranslations.size() > 0)
		return true;
	else
		return false;
}

bool readCameraIntrinsics(string path, Mat & K)
{
	ifstream fin(path + ".\\camparam.txt");
	if (!fin.is_open())
		return false;

	K = Mat(3, 3, CV_32FC1);

	for (int i = 0; i < 9; i++)
		fin >> K.at<float>(i);

	fin.close();
	return true;
}

bool readDepthMap(
	string path,
	Mat &matDepth
)
{
	FileStorage fs(path + ".\\depth.xml", FileStorage::READ);
	fs["depth"] >> matDepth;
	if (matDepth.rows != 0)
		return true;
	else
		return false;
}

void showImage(string windowName, const Mat &img, int delay, int mode)
{
	namedWindow(windowName, mode);
	imshow(windowName, img);
	waitKey(delay);
	destroyWindow(windowName);
}

void normalizeAndRemapToGrayScale(Mat & matIn, Mat &matOut)
{
	matOut = Mat::zeros(matIn.rows, matIn.cols, CV_8UC1);
	float fmin = numeric_limits<float>::max();
	float fmax = numeric_limits<float>::min();

	for (int i = 0; i < matIn.rows*matIn.cols; i++)
	{
		if (matIn.at<float>(i) != 0.f && matIn.at<float>(i) < fmin)
			fmin = matIn.at<float>(i);
		if (matIn.at<float>(i) != 0.f && matIn.at<float>(i) > fmax)
			fmax = matIn.at<float>(i);
	}

	for (int i = 0; i < matIn.rows*matIn.cols; i++)
	{
		if(matIn.at<float>(i)!=0.f)
		matOut.at<uchar>(i) = 255 - (matIn.at<float>(i)-fmin) / (fmax - fmin) * 255;
	}

	//Mat matMean;
	//Mat matStdev;
	//meanStdDev(matIn, matMean, matStdev);

	//float fmean = matMean.at<double>(0, 0);
	//float fstdev = matStdev.at<double>(0, 0);

	//for (int i = 0; i < matIn.rows*matIn.cols; i++)
	//	if (matIn.at<float>(i) == 0.f)
	//		matOut.at<uchar>(i) = 0;
	//	else
	//		matOut.at<uchar>(i) = (1-(matIn.at<float>(i) - fmean) / fstdev) * 255;
}

void visualizePCScores(const Mat & score,int step)
{
	float ratio = step / score.cols;
	Mat matResult(step,score.cols*ratio*2,CV_8UC3);
	int iVal = 0;
	int iValNext = 0;
	for (int i = 0; i < score.rows; i++)
	{
		Scalar color(rand() % 255, rand() % 255, rand() % 255);
		for (int j = 0; j < score.cols-1; j++)
		{
			iVal = step / 2 - score.at<float>(i, j) / (2.0/step);
			iValNext = step / 2 - score.at<float>(i, j+1) / (2.0 / step);
			Point p1(j*ratio * 2, iVal);
			Point p2((j + 1)*ratio * 2, iValNext);
			line(matResult, p1, p2, color, 2);
		}
	}
	//average line
	Scalar red(255, 255, 255);
	for (int j = 0; j < score.cols-1; j++)
	{
		float sum = 0.f;
		float sumNext = 0.f;

		for (int i = 0; i < score.rows; i++)
			sum += score.at<float>(i, j);
		sum /= score.rows;

		for (int i = 0; i < score.rows; i++)
			sumNext += score.at<float>(i, j+1);
		sumNext /= score.rows;

		iVal = step / 2 - sum / (2.0 / step);
		iValNext = step / 2 - sumNext / (2.0 / step);
		Point p1(j*ratio * 2, iVal);
		Point p2((j + 1)*ratio * 2, iValNext);
		line(matResult, p1, p2, red, 2);
	}
	showImage("Vis Responses", matResult, 0);
	imwrite(".\\VisPCScore.png", matResult);
}

float meanf(float * fPtr, int size)
{
	float fmean = 0.f;
	for (int i = 0; i < size; i++)
	{
		fmean += fPtr[i];
	}
	return fmean / size;
}

float stdevf(float * fPtr, int size)
{
	float fmean = meanf(fPtr, size);
	float fresult = 0.f;
	for (int i = 0; i < size; i++)
	{
		fresult += pow(fPtr[i] - fmean, 2);
	}
	return sqrt(fresult/size);
}

float devf(float * fPtr, int size)
{
	float fmean = meanf(fPtr, size);
	float fresult = 0.f;
	for (int i = 0; i < size; i++)
	{
		fresult += pow(fPtr[i] - fmean, 2);
	}
	return fresult / size;
}

float sumf(float * fPtr, int size)
{
	float fresult = 0.f;
	for (int i = 0; i < size; i++)
		fresult +=fPtr[i];
	return fresult;
}

vector<string> getListFiles(string path, string ext, bool addPath)
{
	//fil ehandle
	long   hFile = 0;
	//file info
	struct _finddata_t fileinfo;
	string pathName, exdName;
	vector<string> files;

	if (0 != strcmp(ext.c_str(), ""))
	{
		exdName = "\\*." + ext;
	}
	else
	{
		exdName = "\\*";
	}

	if ((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				files.push_back(pathName.assign(path).append("\\").append(fileinfo.name));
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	return files;
}
