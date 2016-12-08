#pragma once
#include "Util.h"

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