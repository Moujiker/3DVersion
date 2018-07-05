#pragma once
#include <opencv2/core/core.hpp>
#include "dirent.h"
#include <string>
#include <iostream>

using namespace std;
/* A utility function that returns minimum of 3 integers */
inline int min_(int x, int y, int z)
{
	if (x < y)
		return (x < z) ? x : z;
	else
		return (y < z) ? y : z;
}

double LinnerInterpolate(double a, double b, double coefficient)
{
	return a + coefficient * (b - a);
}


// simple wrapper for accessing single-channel matrix elements
 inline uchar getMatElement_uchar(cv::Mat &matrix, int i, int j)
 {
 	assert(matrix.type() == CV_8UC1);
 	if (i < 0 || i >= matrix.rows || j < 0 || j >= matrix.cols)
 		return 0;
 	else
 		return matrix.at<uchar>(i, j);
 }

 //读取文件夹下文件
 int getdir(std::string dir, vector<std::string> &files)
{
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL) 
	{
		cout << "Error(" << errno << ") opening " << dir << endl;
		return errno;
	}
	while ((dirp = readdir(dp)) != NULL) 
	{
		files.push_back(string(dirp->d_name));
	}
	closedir(dp);
	return 0;
}