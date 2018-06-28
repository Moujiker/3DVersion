#include "Image.hpp"
CImage::CImage(const string&fileName, int i, const Mat_<double>& k,const Mat_<double>& rt)
{
    id=i;
    data=imread(fileName);
    int row=data.rows;
    int col=data.cols;
    kmat=k;
    rtmat=rt;
    pmat=k*rt;
    rmat=rt(Rect(0,0,3,3));
    tmat=rt(Rect(3,0,1,3));
    Mat_<double> c=-rmat.t()*tmat;
    Matx<double, 4, 1> hc(c(0,0),c(1,0),c(2,0),1);
    cameraCenter=Mat(hc);
    qf.resize(row-1,vector<set<int>>(col-1));
    qt.resize(row-1,vector<set<int>>(col-1));
    depth.resize(row-1,vector<int>(col-1,-1));
    xaxis(0,0)=rt(0,0);
    xaxis(1,0)=rt(0,1);
    xaxis(2,0)=rt(0,2);
    xaxis(3,0)=0;
    yaxis(0,0)=rt(1,0);
    yaxis(1,0)=rt(1,1);
    yaxis(2,0)=rt(1,2);
    yaxis(3,0)=0;
    zaxis(0,0)=rt(2,0);
    zaxis(1,0)=rt(2,1);
    zaxis(2,0)=rt(2,2);
    zaxis(3,0)=0;

}
void CImage::detectFeatures()
{
    //SIFT sift( 0, 3,0.08, 5, 1.6);
//    SIFT sift;
//    vector<KeyPoint> keypoints;
//    sift(data,Mat(),keypoints,descriptors);
//    for (int i=0; i<keypoints.size(); i++) {
//        Feature f;
//        f.image=this;
//        f.x=keypoints[i].pt.x;
//        f.y=keypoints[i].pt.y;
//        features.push_back(f);
//    }
    vector<Point2d> corners;
    Mat dst;
    cvtColor(data, dst, cv::COLOR_BGR2GRAY);
	//Harris½ÇµãºÍshi-tomasi½Çµã
    goodFeaturesToTrack(dst, corners, 500, 0.02, 10);
    cout<<corners.size()<<endl;
    for (int i=0; i<corners.size(); i++) 
	{
		Feature f;
		f.m_pImage = this;
		f.m_x = corners[i].x;
		f.m_y = corners[i].y;
		m_vecFeature.push_back(f);
    }
    
}

double CImage::getDistanceToCameraCenter(const Mat4x1 &point)const
{
	return norm(cameraCenter - point);
}

Mat3x1 CImage::project(const Mat4x1 &point)const
{
    Mat3x1 m = pmat * point;
    m=m/m(2,0);
    return m;
}
void CImage::grabTexture(const Mat4x1 coord,const Mat4x1&pxaxis,const Mat4x1&pyaxis,Tex&pTex)const
{
    pTex.m_vecPoint.clear();
    pTex.m_vecValue.clear();
    Mat3x1 center=project(coord);
    Mat3x1 dx=project(coord+pxaxis)-center;
    Mat3x1 dy=project(coord+pyaxis)-center;
    
	for (int i = -2; i <= 2; i++)
	{
		for (int j = -2; j <= 2; j++)
		{
			Mat3x1 p = center + j*dx + i*dy;
			Mat v1, v2;
			double px = p(0, 0);
			double py = p(1, 0);
			pTex.m_vecPoint.push_back(Point2d(px, py));
			if (px<0 || px>data.cols - 1 || py<0 || py>data.rows - 1 || std::isnan(px) || std::isnan(py))
			{
				pTex.m_vecValue.push_back(-1);
				pTex.m_vecValue.push_back(-1);
				pTex.m_vecValue.push_back(-1);
			}
			else
			{
				Mat v;
				cv::getRectSubPix(data, Size(1, 1), Point2f(px, py), v);
				pTex.m_vecValue.push_back(v.at<Vec3b>(0, 0)[0]);
				pTex.m_vecValue.push_back(v.at<Vec3b>(0, 0)[1]);
				pTex.m_vecValue.push_back(v.at<Vec3b>(0, 0)[2]);
			}
		}
	}

    pTex.m_Size=norm(4*dx+4*dy);
}
