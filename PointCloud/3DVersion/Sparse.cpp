#include "Sparse.hpp"
#include "Utils.hpp"
#include "Image.hpp"

Sparse::Sparse(const string&fileName,const string&imageDir)
{
    cout<<"loading image..."<<endl;
    loadImages(fileName, imageDir);

	cout << "finish load image & begin detect feature!" << endl;
    for (CImage *image:m_vecImage)
        image->detectFeatures();

	for (int i = 0; i < m_vecImage.size(); i++)
	{
		for (int j = i + 1; j < m_vecImage.size(); j++)
		{
			m_vecImage[i]->m_vecNeighborImage.push_back(m_vecImage[j]);
			m_vecImage[j]->m_vecNeighborImage.push_back(m_vecImage[i]);
		}
     }
	cout << "loading image finish!" << endl;
}
void Sparse::loadImages(const string&fileName,const string&imageDir)
{
    ifstream fin(fileName.c_str());
    int n;
    fin>>n;
    for(int i=0;i<n;i++)
    {
        string name;
        Mat_<double> k(3,3);
        Mat_<double> rt(3,4);
        fin>>name>>k(0,0)>>k(0,1)>>k(0,2)>>k(1,0)>>k(1,1)>>k(1,2)>>k(2,0)>>k(2,1)>>k(2,2)>>rt(0,0)>>rt(0,1)>>rt(0,2)>>rt(1,0)>>rt(1,1)>>rt(1,2)>>rt(2,0)>>rt(2,1)>>rt(2,2)>>rt(0,3)>>rt(1,3)>>rt(2,3);
		//cout << i << "  " << name << endl;
		CImage *pImg = new CImage(imageDir + name, i, k, rt);
		m_vecImage.push_back(pImg);
    }
}

#define SPARSE_DBUBG
void Sparse::buildPatches()
{
    for (CImage *image : m_vecImage) 
	{
        for (const Feature &feature : image->m_vecFeature) 
		{
            if(feature.isInEmptyCell())
            {
                vector<Feature> features;
                feature.findFeaturesNearEpipolarLine(features);
                for_each(features.begin(), features.end(),
						 [&](Feature&f2)
						 {
                             utils::triangluate(feature, f2, f2.m_Point4D);
                         });
                sort(features.begin(), features.end(), [&](const Feature&fa,const Feature&fb)->bool {
                    double d1=fa.getDistanceToCameraCenter(image->cameraCenter);
                    double d2=fa.getDistanceToCameraCenter(fa.m_pImage->cameraCenter);
                    double d3=fb.getDistanceToCameraCenter(image->cameraCenter);
                    double d4=fb.getDistanceToCameraCenter(fb.m_pImage->cameraCenter);
                    if(abs(d1-d2)<abs(d3-d4))
                        return true;
                    else
                        return false;
                });

				cout << "finish Feature detect: " << image->id << endl;
                PPatch bestPatch;
                double bestCost=-1;
                for (const Feature &f2:features) 
				{
                    if(utils::cosangle(f2.m_Point4D, image->cameraCenter, f2.m_pImage->cameraCenter)<=0)
						 continue;

                    PPatch p=PPatch(new Patch());
                    p->m_center=f2.m_Point4D;
                    p->m_normal=image->cameraCenter-f2.m_Point4D;
                    double n=norm(p->m_normal);
                    p->m_normal=p->m_normal/n;
                    p->m_pRefImage=image;
                    p->updateImage(0.4, 0.4);
                    
                    if(p->timages.size()==0)
                        continue;
                    
                    p->optimze();
                    
                    p->updateImage(0.4, 0.7);
                    if(p->timages.size()>=3)
                    {
						double newCost = p->averageCost();
						if (newCost > bestCost)
                        {
                            bestPatch=p;
							bestCost = newCost;
                        }
                    }
                }
				cout << "finish bestPatch: " << image->id << endl;

                if (bestPatch.get()!=NULL) 
				{
                    m_vecPatch.push_back(bestPatch);
					bestPatch->updateImageCell((int)m_vecPatch.size() - 1);
                }
            }
        }
		cout << "finish image: " << image->id << endl;
        savePatches("a.ply");
    }
}

void Sparse::savePatches(const string &fileName)
{
    ofstream fout(fileName);
    
    fout<<"ply"<<endl;
    fout<<"format ascii 1.0"<<endl;
    fout<<"element vertex "<<m_vecPatch.size()<<endl;
    fout<<"property float x"<<endl;
    fout<<"property float y"<<endl;
    fout<<"property float z"<<endl;
    fout<<"property uchar diffuse_red"<<endl;
    fout<<"property uchar diffuse_green"<<endl;
    fout<<"property uchar diffuse_blue"<<endl;
    fout<<"end_header"<<endl;
    for (int i = 0; i < m_vecPatch.size(); i++) 
	{
        Mat4x1 center = m_vecPatch[i]->m_center;
        double r=0,g=0,b=0;
        for(CImage *image : m_vecPatch[i]->timages)
        {
            Mat3x1 x=image->project(center);
            Mat patch;
            getRectSubPix(image->data, Size(1,1), Point2d(x(0,0),x(1,0)), patch);
            b+=patch.at<Vec3b>(0, 0)[0];
            g+=patch.at<Vec3b>(0, 0)[1];
            r+=patch.at<Vec3b>(0, 0)[2];
        }
        r/=m_vecPatch[i]->timages.size();
        g/=m_vecPatch[i]->timages.size();
        b/=m_vecPatch[i]->timages.size();
        fout<<center(0,0)<<" "<<center(1,0)<<" "<<center(2,0)<<" "<<(int)r<<" "<<(int)g<<" "<<(int)b<<endl;
    }
    fout.close();
    cout<<"Finish"<<endl;
}

