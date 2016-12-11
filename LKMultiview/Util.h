#pragma once
#include <iostream>
#include <sstream>
#include <vector>
#include <io.h>

using namespace std;

/*Data Type Conversion*/
int str2int(string str);
float str2float(string str);
double str2double(string str);
string int2str(int i);
string float2str(float f);
string double2str(double d);

/*IO*/
vector<string> getListFiles(string path, string extension, bool addPath);