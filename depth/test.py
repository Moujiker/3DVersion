# -*- coding:utf-8 -*-
import os
import cv2
import numpy as np
from matplotlib import pyplot as plt

def testDepthMap():
	ImageFile = 'data/'
	validFile = ['jpg','png','JPG']
	listAllImage = os.listdir(ImageFile)
	Imagelist = filter(lambda x : x.split('.')[-1] in validFile,listAllImage)
	Imagelist = map(lambda x : cv2.imread(ImageFile+x),Imagelist)

	imageSize = Imagelist[0].shape
    #print "imageSize:",imageSize

	'''
    #如果大于maxSize, 缩放图像
    if imageSize[0] > maxSize:
    	print "Size image ",imageSize," max size ",maxSize
    	Imagelist = map(lambda x: np.transpose(cv2.resize(x,(640,480)),axes=[1,0,2]), Imagelist)
    	imageSize = Imagelist[0].shape
    	print "After resize image size ",imageSize
	'''
def testExample():
	left_img = cv2.imread('l.jpg', 0)
	right_img = cv2.imread('r.jpg', 0)# 为什么必须是灰阶图像
	matching = cv2.StereoBM(1, 16, 19)#参数意义
	depth_map = matching.compute(left_img, right_img)
	#cv2.imwrite('depth_map.png', depth_map)
	#cv2.imshow("Image", right_img) 
	#cv2.waitKey(0)
	plt.imshow(depth_map,'gray')
	plt.show()

if __name__ == '__main__':
	testExample()