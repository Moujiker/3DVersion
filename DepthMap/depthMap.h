#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "frameParams.h"

class depthMap
{
public:
	depthMap(size_t width,  size_t height);
	~depthMap();

public:
	void readDepthmaps();
	void readImgs();

	void depthFusion();

	void depth2model(int index);
	cv::Mat_<double> model2depth(std::string modelName, int index);

	// ”≤ÓÕº->…Ó∂»Õº
	//https://www.cnblogs.com/riddick/p/8486223.html


	//https://www.cnblogs.com/cv-pr/p/5719350.html
	//https://blog.csdn.net/u012423865/article/details/78036543?locationNum=3&fps=1
	void Depth2PointCloud(cv::Mat_<double> imDepth, std::string filePath);

	void test(std::string dataPath);

	//grayscale depth images to 1:1 meshes and point sets
	// only exports to .xyz point sets without color info
	bool writeDepthToMeshfile(const char* fileName, const unsigned char* depthData, const unsigned char* colorData, bool cullBlack, unsigned int minMM = 720, unsigned int maxMM = 900);
private:
	size_t m_width;
	size_t m_height;

	std::string m_dataPath;

	std::vector<cv::Mat_<double>> m_vecDepthmaps;
	std::vector<cv::Mat> m_vecImages;
	KeyframeSelection m_keyFrameSel;
};
