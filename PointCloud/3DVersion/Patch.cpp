//
//  Patch.cpp
//  Document
//
//  Created by  刘骥 on 16/4/26.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "Patch.hpp"
#include "Utils.hpp"

#define M_PI_2   3.14    

#pragma comment(lib,".\\nlopt-2.4.2-dll64\\libnlopt-0.lib")

void Patch::updateImage(double alpha1,double alpha2)
{
    timages.clear();
    simages.clear();
    Tex pTex1;
    Mat4x1 pxaxis,pyaxis;
    getPAxes(pxaxis, pyaxis);
    m_pRefImage->grabTexture(m_center,pxaxis,pyaxis,pTex1);
    
    for(CImage *nimage:m_pRefImage->m_vecNeighborImage)
    {
        if(utils::cosangle(m_center, m_pRefImage->cameraCenter, nimage->cameraCenter)<0)
           continue;
        Tex pTex2;
        nimage->grabTexture(m_center, pxaxis, pyaxis, pTex2);
        double v=pTex1.NCC(pTex2);
        
        if (v>alpha1) 
            simages.push_back(nimage);
    
        if(v>alpha2)
            timages.push_back(nimage);
    }
}

//detph：patch中心到相机的距离
//alpha：patch法线与相机Y轴的夹角(-π/2,π/2)
//beta：patch法线在XOZ平面上的投影与相机Z轴的夹角(-π/2,π/2)
void Patch::encode(double &depth, double &alpha, double &beta)const
{
    depth=m_pRefImage->getDistanceToCameraCenter(m_center);
    
    double fx=m_pRefImage->xaxis.dot(m_normal);
    double fy=m_pRefImage->yaxis.dot(m_normal);
    double fz=m_pRefImage->zaxis.dot(m_normal);
    alpha=asin(fy);
    double cosAlpha=cos(alpha);
    double sinBeta=fx/cosAlpha; //fx=sinβ*cosα
    double cosBeta=-fz/cosAlpha; //-fz=cosβ*cosα
    beta=acos(cosBeta);
    if (sinBeta<0) {
        beta=-beta;
    }    
}

void Patch::decode(double depth, double alpha, double beta)
{
   
    double fx=sin(beta)*cos(alpha);
    double fy=sin(alpha);
    double fz=-cos(beta)*cos(alpha);
    
    m_normal=fx*m_pRefImage->xaxis+fy*m_pRefImage->yaxis+fz*m_pRefImage->zaxis;
    m_normal(3,0)=0;
    m_center=m_pRefImage->cameraCenter+m_ray*depth;
}

void Patch::getPAxes(Mat4x1 &pxaxis, Mat4x1 &pyaxis)const
{
    CImage &image=*m_pRefImage;
    Vec3d zaxis(m_normal(0,0),m_normal(1,0),m_normal(2,0));
    Vec3d xaxis=Vec3d(image.xaxis(0,0),image.xaxis(1,0),image.xaxis(2,0));
    Vec3d yaxis=zaxis.cross(xaxis);
    yaxis=yaxis/norm(yaxis);
    xaxis=yaxis.cross(zaxis);
    double depth=norm(m_center-image.cameraCenter);
    double scale=2*depth/(image.kmat(0,0)+image.kmat(1,1));
    pxaxis(0,0)=xaxis[0];
    pxaxis(1,0)=xaxis[1];
    pxaxis(2,0)=xaxis[2];
    pxaxis(3,0)=0;
    pyaxis(0,0)=yaxis[0];
    pyaxis(1,0)=yaxis[1];
    pyaxis(2,0)=yaxis[2];
    pyaxis(3,0)=0;
    pxaxis=pxaxis*scale;
    pyaxis=pyaxis*scale;
    double xdis=norm(image.project(m_center+pxaxis)-image.project(m_center));
    double ydis=norm(image.project(m_center+pyaxis)-image.project(m_center));
    pxaxis=pxaxis/xdis;
    pyaxis=pyaxis/ydis;
}

double Patch::averageCost()const
{
    if (timages.size()==0)
        return -1;

    Tex pTex1;
    Mat4x1 pxaxis,pyaxis;
    getPAxes(pxaxis, pyaxis);
    m_pRefImage->grabTexture(m_center,pxaxis,pyaxis,pTex1);
    
    double sum=0;
    //TODO nimages还是timages？
    //for(Image *timage:timages)
	for ( auto timage : timages)
    {
        Tex pTex2;
        timage->grabTexture(m_center, pxaxis, pyaxis, pTex2);
        sum+=cost(pTex1,pTex2);
    }
    double ret=sum/timages.size();
    if(std::isnan(ret)||std::isinf(ret))
        return -1;
    return ret;
}
double Patch::cost(const Tex&tex1,const Tex&tex2) const
{
    return tex1.NCC(tex2);
}
double optimizeFun(const vector<double> &x, vector<double> &grad, void *fdata)
{
    Patch p;
    Patch *oldP=(Patch*)fdata;
    p.timages=oldP->timages;
    p.m_pRefImage=oldP->m_pRefImage;
    p.m_ray=p.m_ray;
    p.decode(x[0], x[1], x[2]);
    return p.averageCost();
    
}
void Patch::optimze()
{
    double depth,alpha,beta;
    m_ray=m_center-m_pRefImage->cameraCenter;
    m_ray=m_ray/norm(m_ray);

    encode(depth, alpha, beta);

    nlopt::opt opt(nlopt::LN_BOBYQA,3);
    opt.set_max_objective(optimizeFun, this);
    opt.set_xtol_rel(1.e-7);
    opt.set_maxeval(100);
    vector<double> x(3),lb(3),ub(3);
    lb[0]=0.1; //HUGE_VAL极大值
    lb[1]=-M_PI_2/3; //-pi/2
    lb[2]=-M_PI_2/3; //-pi/2
    ub[0]=HUGE_VAL;
    ub[1]=M_PI_2/3;
    ub[2]=M_PI_2/3;
    x[0]=depth;
    x[1]=alpha;
    x[2]=beta;
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    bool success=false;
    double maxf=averageCost();
    try{
        nlopt::result result=opt.optimize(x, maxf);
        success = (result == nlopt::SUCCESS
                   || result == nlopt::STOPVAL_REACHED
                   || result == nlopt::FTOL_REACHED
                   || result == nlopt::XTOL_REACHED);
        //cout<<"maxf:"<<maxf<<endl;
    }catch(std::exception &e)
    {
        success = false;
    }
    if (success) 
	{
        decode(x[0], x[1], x[2]);
    }
    
}
void Patch::updateImageCell(int pid)
{
    Tex pTex1;
    Mat4x1 pxaxis,pyaxis;
    getPAxes(pxaxis, pyaxis);
    m_pRefImage->grabTexture(m_center,pxaxis,pyaxis,pTex1);
    pTex1.updateCell(pid, m_pRefImage->qt);
}
void Patch::showResult()
{
    CImage &image1=*m_pRefImage;
    Mat4x1 pxaxis,pyaxis;
    getPAxes(pxaxis, pyaxis);
    Tex pTex1;
    image1.grabTexture(m_center, pxaxis, pyaxis, pTex1);
    Mat im1,im2;
    image1.data.copyTo(im1);
    for (int i=0; i<pTex1.m_vecPoint.size(); i++) {
        cv::circle(im1, pTex1.m_vecPoint[i],1, Scalar(0,0,255));
    }
    cv::namedWindow("im1");
    cv::namedWindow("im2");
    cv::imshow("im1", im1);
    for (int i=0; i<timages.size(); i++) {
        Tex pTex2;
        CImage &image2=*timages[i];
        image2.grabTexture(m_center, pxaxis, pyaxis, pTex2);
        image2.data.copyTo(im2);
        cout<<pTex1.NCC(pTex2)<<endl;
        cout<<"array[";
        for_each(pTex1.m_vecValue.begin(), pTex1.m_vecValue.end(), [](double v){
            cout<<v<<",";
        });
        cout<<"]"<<endl;
        cout<<"array[";
        for_each(pTex2.m_vecValue.begin(), pTex2.m_vecValue.end(), [](double v){
            cout<<v<<",";
        });
        cout<<"]"<<endl;
        for (int j=0; j<pTex2.m_vecPoint.size(); j++) {
            //cout<<pTex2.points[j]<<endl;
            cv::circle(im2, pTex2.m_vecPoint[j],1, Scalar(0,0,255));
        }
        cv::imshow("im2",im2);
        cv::waitKey();
        
    }

}
