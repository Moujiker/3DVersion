#include "Tex.hpp"

//Normalized Cross Correlation
double Tex::NCC(const Tex &other)const
{
    double ratio=other.m_Size>m_Size? m_Size/other.m_Size : other.m_Size/m_Size;
    if(std::isnan(ratio))
        return -1;
    double m1=mean(m_vecValue)[0];
    double m2=mean(other.m_vecValue)[0];
    double m=m1*m_vecValue.size()+m2*other.m_vecValue.size();
    m=m/(m_vecValue.size()*2);
    double n1=0;
    double n2=0;
    for (int i=0; i<m_vecValue.size(); i++) 
	{
        if (m_vecValue[i]==-1 ||other.m_vecValue[i]==-1)
            return -1;

        n1+=pow(m_vecValue[i]-m,2);
        n2+=pow(other.m_vecValue[i]-m,2);
    }
    n1=sqrt(n1);
    n2=sqrt(n2);
    if(n1==0||n2==0)
        return -1;
    double sum=0;
    for (int i=0; i<m_vecValue.size(); i++)
	{
        if (m_vecValue[i]==-1 ||other.m_vecValue[i]==-1) 
		{
            return -1;
        }
        sum+=(m_vecValue[i]-m)*(other.m_vecValue[i]-m);
    }
    return sum/(n1*n2);
}

void Tex::updateCell(int id,vector<vector<set<int> > >&cell)
{
    int minX,minY,maxX,maxY;
    getMinMaxXMinMaxY(minX,maxX,minY,maxY);

    for(int i = minX;i < maxX; i++)
        for(int j = minY;j < maxY; j++)
            cell[j][i].insert(id);
}

void Tex::getMinMaxXMinMaxY(int& minX,int& maxX,int& minY,int& maxY)
{
	minY = minX = INT_MAX;
	maxY = maxX = INT_MIN;
    for (vector<Point2d>::iterator it=m_vecPoint.begin();it!=m_vecPoint.end();it++) 
	{
        if(floor(it->x)<minX)
        {
            minX=floor(it->x);
        }
        if(ceil(it->x)>maxX)
        {
            maxX=ceil(it->x);
        }
        if(floor(it->y)<minY)
        {
            minY=floor(it->y);
        }
        if(ceil(it->y)>maxY)
        {
            maxY=ceil(it->y);
        }
    }
}
