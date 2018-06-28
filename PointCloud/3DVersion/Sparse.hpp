#ifndef Sparse_hpp
#define Sparse_hpp
#include <vector>
#include <opencv2/opencv.hpp>
#include "Image.hpp"
#include "Patch.hpp"
#include "Tex.hpp"
#include <fstream>
#include <string>
using namespace std;
using namespace cv;
class Sparse
{
public:
    Sparse(const string&fileName,const string&imageDir);
    void buildPatches();
    void savePatches(const string&fileName);

private:
	void loadImages(const string&fileName, const string&imageDir);

private:
	vector<CImage *> m_vecImage;
	vector<PPatch> m_vecPatch;

};
#endif /* Sparse_hpp */
