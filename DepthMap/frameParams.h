#ifndef FRAMEPARAMS_H
#define FRAMEPARAMS_H

#include <iostream>
#include <fstream>

#include <Eigen/Eigen>

#include <vector>

#define PI 3.14159265897
class CCamera;

class KeyframeSelection 
{
public:
	std::string dataPath;
	
	void test(std::string path, int ref);
	
	Eigen::Vector3d getViewVector(int refCamIndex, int x, int y);
	Eigen::Vector3d get3Dpoint(int refCamIndex, int x, int y, double depth);

	int referCamIndex;
	//angleDegree * Pi / 180 = angleRadian
	double maxAngleRadian, minAngleRadian, maxAngleDegree, minAngleDegree, maxBaseline, minBaseline;

	std::vector<int> idxOfKeyframe;

	void readParams();

	std::vector<CCamera> parameters;
private:

	void selectKeyframes();

	int index; //number of params
};

#endif