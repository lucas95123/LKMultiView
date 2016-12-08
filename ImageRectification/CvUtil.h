#pragma once
#include <fstream>

#include "CameraParameter.h"
#include "Exceptions.h"

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

/*IO*/
void readCameraParameters(map<string, CameraParameter> &mapCameraParam, string path);

/*Print Utility*/
void printArray(const float *pfArr, int iCount);
void printMat(const Mat &refMat);

/*Image Utility*/
void showImage(string windowName,const Mat &img);

/*OpenCV Conversions*/
void pointPairToPoints(const vector<pair<Point2f, Point2f>> &pairs, vector<Point2f> &points1, vector<Point2f> &points2);