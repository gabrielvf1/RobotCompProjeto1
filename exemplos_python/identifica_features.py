#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
MIN_MATCH_COUNT=60

#detector=cv2.ORB_create()
detector=cv2.xfeatures2d.SIFT_create()	
#detector=cv2.BRISK_create()
#detector=cv2.AKAZE_create()


FLANN_INDEX_KDITREE=0
##flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=3)

index_params = dict(algorithm = FLANN_INDEX_KDITREE, trees = 5)
search_params = dict(checks =  60)
flann=cv2.FlannBasedMatcher(index_params,search_params)


#flann = cv2.BFMatcher()


#trainImg=cv2.imread("caixa2.jpg",0)
trainImg=cv2.imread("imagens/foto_aprendida.jpg",0)
rospy.sleep(2)
trainKP,trainDesc=detector.detectAndCompute(trainImg,None)
h,w=trainImg.shape



def identifica_feat(frame):
	#print('1')

	QueryImgBGR=frame

	QueryImg=cv2.cvtColor(QueryImgBGR,cv2.COLOR_BGR2GRAY)
	#print('2')


	# queryKP=detector2.detect(QueryImgBGR,None)
	# queryKP,queryDesc=detector2.compute(QueryImgBGR,queryKP)



	queryKP,queryDesc=detector.detectAndCompute(QueryImgBGR,None)
		



	#print(len(queryDesc))
	#print('3')
	matches=flann.knnMatch(queryDesc,trainDesc,k=2)
	#print('4')
	#queryKP,queryDesc=[1,1]
	

	goodMatch=[]
	for m,n in matches:
		if(m.distance<0.75*n.distance):
			goodMatch.append(m)
			
	if(len(goodMatch)>MIN_MATCH_COUNT):
		# print("matches suficientes:{0}".format(len(goodMatch)))
		tp=[]
		qp=[]
		
		
		
		for m in goodMatch:
			tp.append(trainKP[m.trainIdx].pt)
			qp.append(queryKP[m.queryIdx].pt)
		tp,qp=np.float32((tp,qp))
		H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
		
		trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
		queryBorder=cv2.perspectiveTransform(trainBorder,H)
	
		cv2.polylines(QueryImgBGR,[np.int32(queryBorder)],True,(0,255,0),5)
		

		query_temp=[]

		for i in range(len(queryBorder[0])):
			query_temp.append([queryBorder[0][i][0],queryBorder[0][i][1]])


		distancia_AB = math.sqrt( (query_temp[0][0] - query_temp[1][0])**2 + (query_temp[0][1] - query_temp[1][1])**2 )
		distancia_BC = math.sqrt( (query_temp[1][0] - query_temp[2][0])**2 + (query_temp[1][1] - query_temp[2][1])**2 )

		maior_contorno_area = distancia_AB * distancia_BC


		media_x = (query_temp[0][0] + query_temp[1][0] + query_temp[2][0] + query_temp[3][0])/4 
		media_y = (query_temp[0][1] + query_temp[1][1] + query_temp[2][1] + query_temp[3][1])/4

		media = [media_x,media_y] 
	


			
	else:
		# print("matches insuficientes:{0}".format(len(goodMatch)))
		#cv2.imshow('result',QueryImgBGR)
		media = (0,0)
		maior_contorno_area = 0

	cv2.imshow('result',QueryImgBGR)
	cv2.waitKey(10)

	centro = (QueryImgBGR.shape[0]//2, QueryImgBGR.shape[1]//2)
	GOODMATCH=len(goodMatch)

	return media, centro, maior_contorno_area, GOODMATCH