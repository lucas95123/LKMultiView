#pragma once
#include <iostream>
#include <string>
#include <opencv2\core.hpp>

using namespace std;

class BaseException : public cv::Exception
{
protected:
	string m_strExceptionType = "BaseException";
	string m_strExceptionMesg;
public:
	BaseException() {}
	BaseException(string mesg) {
		m_strExceptionMesg = mesg;
	}
	string what()
	{
		return m_strExceptionType + ": " + m_strExceptionMesg;
	}
};

class FileNotFoundException : public BaseException
{
public:
	FileNotFoundException(string mesg) :BaseException(mesg)
	{
		m_strExceptionType = "FileNotFoundException";
	}
};

class InvalidFormatException : public BaseException
{
public:
	InvalidFormatException(string mesg) :BaseException(mesg)
	{
		m_strExceptionType = "InvalidFormatException";
	}
};