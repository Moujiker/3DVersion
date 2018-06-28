#ifndef Tex_hpp
#define Tex_hpp
#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>
using namespace std;
using namespace cv;
class Tex
{
public:
	double NCC(const Tex& other)const;
	void getMinMaxXMinMaxY(int& minX, int& maxX, int& minY, int& maxY);
	void updateCell(int id, vector<vector<set<int> > >& cell);

public:
    int m_id;
    vector<Point2d> m_vecPoint;
    vector<double> m_vecValue;
    double m_Size;
};
#endif /* Tex_hpp */
