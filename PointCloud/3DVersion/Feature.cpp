#include "Feature.hpp"
#include "Utils.hpp"

bool Feature::isInEmptyCell()const
{    
	return m_pImage->qt[(int)m_y][(int)m_x].empty()&&m_pImage->qf[(int)m_y][(int)m_x].empty();
}

Mat3x1 Feature::toHomogeneous()const
{
    Mat3x1 m;
    m(0,0)=m_x;
    m(1,0)=m_y;
    m(2,0)=1;
    return m;
}

double Feature::getDistanceToCameraCenter(const Mat4x1 &cameraCenter)const
{
    return norm(cameraCenter-m_Point4D);
}

void Feature::findFeaturesNearEpipolarLine(vector<Feature>&featuresNearEpipolarLine)const
{
    for (CImage *nimage : m_pImage->m_vecNeighborImage) 
	{
        Mat3x1 line;
        utils::computeEpipolarLine(m_pImage->pmat,nimage->pmat,m_pImage->cameraCenter,*this,line);
        double a=line(0,0);
        double b=line(1,0);
        double c=line(2,0);
        for (const Feature &f:nimage->m_vecFeature) 
		{
            if(f.isInEmptyCell())
            {
				double d = abs(a*f.m_x + b*f.m_y + c) / sqrt(a*a + b*b);
				if (d <= 1)
				{
					featuresNearEpipolarLine.push_back(f);
				}
            }
        }
    }
}
