#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "Image.hpp"
using namespace cv;
using namespace std;
void loadImages(const string&fileName,const string&dir,vector<CImage>&images)
{
    ifstream fin((dir+fileName).c_str());
    int n;
    fin>>n;
    for(int i=0;i<n;i++)
    {
        string name;
        Mat_<double> k(3,3);
        Mat_<double> rt(3,4);
        fin>>name>>k(0,0)>>k(0,1)>>k(0,2)>>k(1,0)>>k(1,1)>>k(1,2)>>k(2,0)>>k(2,1)>>k(2,2)>>rt(0,0)>>rt(0,1)>>rt(0,2)>>rt(1,0)>>rt(1,1)>>rt(1,2)>>rt(2,0)>>rt(2,1)>>rt(2,2)>>rt(0,3)>>rt(1,3)>>rt(2,3);
        images.push_back(CImage(dir+name,i,k,rt));
    }
}

void buildVisData(const vector<CImage>&images, vector<vector<int>>& vis,double ratio=2,int threshold=15)
{
    BFMatcher bf;
    vector<vector<DMatch> > knnMatches;
    for (int i=0;i<images.size(); i++) {
        for(int j=i+1;j<images.size();j++)
        {
            
            bf.knnMatch(images[i].descriptors,images[j].descriptors,knnMatches,2);
            int count=0;
            for(vector<vector<DMatch> >::iterator it=knnMatches.begin();it!=knnMatches.end();it++)
            {
                if((*it)[1].distance/(*it)[0].distance>ratio)
                    count++;
            }
            if(count>threshold)
            {
                vis[i][j]=j;
                vis[j][i]=i;
            }
            
        }
    }
}

int main_t(int argc,char *argv[])
{
    string fileName=argv[1];
    string dir=argv[2];
    cout<<"加载图像中..."<<endl;
    vector<CImage> images;
    loadImages(fileName, dir,images);
    for (CImage &image:images) {
        image.detectFeatures();
    }
    cout<<"计算图像邻居.."<<endl;
    vector<vector<int>> vis(images.size(),vector<int>(images.size(),-1));
    buildVisData(images,vis);
    for(int i=0;i<vis.size();i++)
    {
        cout<<i<<" ";
        for(int j=0;j<vis[i].size();j++)
        {
            if(vis[i][j]!=-1)
            cout<<vis[i][j]<<" ";
        }
        cout<<endl;
    }
    return 0;
}