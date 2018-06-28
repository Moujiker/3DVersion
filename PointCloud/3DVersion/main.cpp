//  main.cpp
//  Document
//  Created by  刘骥 on 16/4/26.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "Image.hpp"
#include "Patch.hpp"
#include "Feature.hpp"
#include <fstream>
#include <vector>
#include "Sparse.hpp"
using namespace std;

int main(int argc,char* argv[])
{
	//Sparse sparse(argv[1],argv[2]);
	Sparse sparse("./images/temple2/temple_par.txt","./images/temple2/");
    
   // Sparse sparse("./images/templeRing/templeR_par.txt","./images/templeRing/");
   // Sparse sparse("./images/templeSparseRing/templeSR_par.txt","./images/templeSparseRing/");
    sparse.buildPatches();
	std::cout <<  "finish build Patch" << endl;
	cv::waitKey(0);
	return 0;
}
