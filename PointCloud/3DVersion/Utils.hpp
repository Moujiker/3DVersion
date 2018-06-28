#ifndef UTILS_HPP
#define UTILS_HPP
#include<opencv2/opencv.hpp>
#include<cmath>
#include "Feature.hpp"
#include "Matmxn.hpp"
using namespace std;
namespace utils
{
    inline void triangluate(const Feature&f1,const Feature&f2,Mat4x1&point)
    {
		Mat points4D;
        vector<Point2d> points1;
        points1.push_back(Point2d(f1.m_x,f1.m_y));
        vector<Point2d> points2;
        points2.push_back(Point2d(f2.m_x,f2.m_y));
		//使用三角测量重建点, 投影矩阵可以从stereorectify()获得
        triangulatePoints(f1.m_pImage->pmat, f2.m_pImage->pmat,points1,points2,points4D);
        point=points4D;
        point=point/point(3,0);
        point(3,0)=1;
    }

    inline double cosangle(const Mat&point,const Mat&c1,const Mat&c2)
    {
        Mat v1=point-c1;
        Mat v2=point-c2;
        double ret=v1.dot(v2)/(norm(v1)*norm(v2));
        if(std::isnan(ret))
            return -1;
        return ret;
    }

    inline void computeEpipolarLine(const Mat3x4 &p1,const Mat3x4 &p2, const Mat4x1&c1,const Feature &f, Mat3x1 &line)
    {
        //Mat3x4 p1=pmat;
        //Mat4x1 c1=cameraCenter;
        Mat3x3 pinvP1=p1.t()*(p1*p1.t()).inv();
        Mat3x1 x;
        x(0,0)=f.m_x;
        x(1,0)=f.m_y;
        x(2,0)=1;
        line=(p2*c1).cross(p2*pinvP1*x);
    }
}
#endif
