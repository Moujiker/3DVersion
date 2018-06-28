#ifndef Image_hpp
#define Image_hpp
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
#include "Feature.hpp"
#include "Tex.hpp"
#include "Matmxn.hpp"
using namespace std;
using namespace cv;
class Feature;
class CImage
{
public:
    CImage(const string&fileName,int i,const Mat_<double>& k,const Mat_<double>& rt);
	~CImage(){}

	void   detectFeatures();
	Mat3x1 project(const Mat4x1& point)const;
	double getDistanceToCameraCenter(const Mat4x1& point)const;

	//获取图像纹理
	void   grabTexture(const Mat4x1 coord, const Mat4x1& pxaxis, const Mat4x1& pyaxis, Tex& pTex)const;

public:
    int id;
    Mat data;
    Mat3x3 kmat;
    Mat3x4 pmat; //相机的投影矩阵
    Mat3x4 rtmat; //旋转平移矩阵
    Mat3x3 rmat;
    Mat3x1 tmat;
    Mat4x1 cameraCenter;
    Mat4x1 xaxis;
    Mat4x1 yaxis;
    Mat4x1 zaxis;
    vector<vector<set<int> > > qf; //Qf(i,j)
    vector<vector<set<int> > > qt; //Qt(i,j)
    vector<vector<int> > depth; //depth(i,j)
    vector<Feature> m_vecFeature;
    Mat descriptors;
    vector<CImage*> m_vecNeighborImage;
};
#endif /* Image_hpp */
