#pragma once
#include <fstream>

#include "CameraParameter.h"
#include "Exceptions.h"

#include <opencv2\opencv.hpp>

/*IO*/
void readCameraParameters(vector<CameraParameter> &mapCameraParam, string path);

void writeMat(Mat &matData, string strPath);

/*Print Utility*/
void printArray(const float *pfArr, int iCount);

void printMat(const Mat &refMat);

void writeMat(ofstream &fout, const Mat &refMat);

/*Image Utility*/
void showImage(string windowName,const Mat &img);

/*OpenCV Conversions*/
void pointPairToPoints(const vector<pair<Point2f, Point2f>> &pairs, vector<Point2f> &points1, vector<Point2f> &points2);

void decomposeEssentialMatrix(Mat &E, Mat &R, Mat &T);

bool isRotationMatrix(Mat &R);

Vec3f rotationMatrixToEulerAngles(Mat &R);