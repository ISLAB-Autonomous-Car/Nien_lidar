import rospy
import cv2
import numpy as np
import time
from std_msgs.msg import String, Float32, Int32, Float32MultiArray, Bool
import sensor_msgs.msg
from geometry_msgs.msg import Twist
from itertools import *
import sys
import config as cf
import rospkg 
import threading
from operator import itemgetter
#from tracker import Tracker

#import imutils
import math
de2ra = 0.0174533
time_fps = time.time()

class lidar_pro:
	def __init__(self):
		self.data_lidar = np.zeros(shape=[2, 360], dtype = np.float)
        # 2 kích cỡ hình 
		self.image_draw = np.zeros((320, 240, 3), np.uint8)
		self.image_draw2 = np.zeros((280, 460, 3), np.uint8)
		self.carX = 120#240#60 # hệ số dùng để đem lidar về center
		self.carY = 240#500#100
		self.angle_min = 150#100 # >0
		self.angle_max = -150#-100 # <0
		self.scale = 100
		self.bphai = 0.5
		self.btrai = 0.1
		self.bxa = 2.0
		self.bsau = 0.3
		self.yes_obstacle = 0
		self.array = None
		self.sub_lidar_data = rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan, self.callback_lidar, queue_size = 1)
	def talker(self, data):
	  	pub1 = rospy.Publisher('/obstacle', Int32, queue_size=1)
	  	rate = rospy.Rate(100000)
	  	if (~rospy.is_shutdown()):
	  		pub1.publish(data)
	  		rate.sleep()
	def draw_map(self):
		global de2ra
		x0 = self.carX
		y0 = self.carY
		angle_min = self.angle_min
		angle_max = self.angle_max
		scale = self.scale
		image = np.zeros((self.image_draw.shape[0], self.image_draw.shape[1]), np.uint8)
		img = np.zeros((self.image_draw2.shape[0], self.image_draw2.shape[1]), np.uint8)
		dist = self.data_lidar
		
		phai = int(self.bphai*self.scale)
		trai = int(self.btrai*self.scale)
		xa = int(self.bxa*self.scale)
		sau = int(self.bsau*self.scale)
		x1 = x0-trai
		y1 = y0-xa
		x2 = x0+phai
		y2 = y0+sau
		count = 0
		for i in range(angle_min, angle_max, -1):
			x = 0
			y = 0				
			if dist[0][i] < 100:
				x = int(x0 - scale*dist[0][i]*math.sin(dist[1][i]*de2ra))
				y = int(y0 - scale*dist[0][i]*math.cos(dist[1][i]*de2ra))
				#cv2.circle(image, (x, y), 5, (255, 255, 255), -1)
				if x1 <= x <= x2 and y1 <= y <= y2:
					image[y][x] = 255
					count += 1
		#print("count:   ",count)
		if count < 5:
			self.yes_obstacle = 0
			return 0
		# cut ROI

		mask_roi2 = np.ones((image.shape[0], image.shape[1]), np.uint8)*255 
		roi2 = np.array([[(x0-15, y0+15), (x0-15, y0), (x0+15, y0), (x0+15, y0+15)]], dtype=np.int32)
		mask_roi2 = cv2.fillPoly(mask_roi2, roi2, (0, 0, 0))
		image = cv2.bitwise_and(image, mask_roi2)
		
		
		hough = cv2.HoughLines(image,1,1*np.pi/180,2)
		if hough is None:
			self.yes_obstacle = 0
			return 0
		#if len(hough) < 10:
		#	self.yes_obstacle = 0
		#	return 0

		cv2.line(image, (int(x0-trai), int(y0+sau)), (int(x0-trai), int(y0-xa)), (255,255,255), 2)
		cv2.line(image, (int(x0-trai), int(y0-xa)), (int(x0+phai), int(y0-xa)), (255,255,255), 2)
		cv2.line(image, (int(x0+phai), int(y0-xa)), (int(x0+phai), int(y0+sau)), (255,255,255), 2)
		cv2.line(image, (int(x0+phai), int(y0+sau)), (int(x0-trai), int(y0+sau)), (255,255,255), 2)
		self.image_draw = image
		self.image_draw2 = img

		#print("num point", len(hough))
		for h in hough:
			rho,theta=h[0]
			rho1=rho+250
			theta1= theta*180/np.pi 
			cv2.circle(img, (int(rho1), int(theta1)), 5, (255,255,255), 5)

		contours,_ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
		xmin=400
		arr=[]
		for cnt in contours:
			x,y,w,h = cv2.boundingRect(cnt)
			print(w, h)
			if y > 10:
				arr.append([x,y,w,h])
		#print("arr" ,len(arr))
		if len(arr) > 1:
			self.yes_obstacle = 2
			return 0
		else:
			self.yes_obstacle = len(arr)
			return 0
				
		#print("arr", arr[-1])
	def callback_lidar(self,data):
		global time_fps
		fps = int(1/(time.time()-time_fps))
		time_fps = time.time() 
		#print("FPS", data)
		#range_angels = np.arange(len(data.ranges))
		for i in range(360):
			d = data.ranges[i]
			if d == float("inf"):
				d = 100
			self.data_lidar[0][i] = d
			if i < 180:
				self.data_lidar[1][i] = i
			elif i > 180:
				self.data_lidar[1][i] = i - 360
			else:
				self.data_lidar[1][i] = 0
		self.draw_map()
		self.talker(self.yes_obstacle)
		print("num box", self.yes_obstacle)
		#cv2.imshow('lidar1', self.image_draw)
		#cv2.imshow('lidar2', self.image_draw2)
		k = cv2.waitKey(5)
		

if __name__ == '__main__':
	try:
		rospy.init_node('lidar_stream')
		rospy.loginfo("start")
		lidar_pro()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()