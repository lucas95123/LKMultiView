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
