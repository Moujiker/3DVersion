#ifndef Feature_hpp
#define Feature_hpp
#include <opencv2/opencv.hpp>
#include "Image.hpp"
#include "Matmxn.hpp"
using namespace cv;
class CImage;
class Feature
{

public:
	bool   isInEmptyCell()const;
	Mat3x1 toHomogeneous()const;

	//distance to camera center
	double getDistanceToCameraCenter(const Mat4x1&cameraCenter)const;
	void   findFeaturesNearEpipolarLine(vector<Feature>& featuresNearEpipolarLine)const;

public:
    double m_x;
    double m_y;
	Mat4x1 m_Point4D;
	CImage *m_pImage;
};
#endif /* Feature_hpp */
