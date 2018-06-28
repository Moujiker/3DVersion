#ifndef Patch_hpp
#define Patch_hpp
#include <opencv2/opencv.hpp>
#include <vector>
#include "nlopt.hpp"
#include <memory>
#include "Image.hpp"
#include "Matmxn.hpp"
using namespace cv;
using namespace std;
class Patch
{
public:
    Mat4x1 m_center;
    Mat4x1 m_normal;
    Mat4x1 m_ray;
    CImage* m_pRefImage; //reference image
    vector<CImage*> simages; //S(p)
    vector<CImage*> timages; //T(p)
    void encode(double &depth, double &alpha, double &beta)const;
    void decode(double depth, double alpha, double beta);
    void getPAxes( Mat4x1 &pxaxis, Mat4x1 &pyaxis)const ;
    double averageCost()const;
    double cost(const Tex&tex1,const Tex&tex2) const;
    void optimze();
    void updateImage(double alpha1,double alpha2);
    void updateImageCell(int pid);
    void showResult();
};
typedef  shared_ptr<Patch> PPatch;
#endif /* Patch_hpp */
