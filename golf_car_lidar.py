from matplotlib.colors import LinearSegmentedColormap
from rplidar import RPLidar
import numpy as np
import cv2
import math
import time
from Lidar_cluster import cluster
class Lidar_Golf_car:
    def __init__(self,port_name,img_size,error):
        #self.Car = (img_size[1]/2,img_size[0]/2)
        self.Car = (0,0)
        self.scale = 1
        self.alpha = (2*math.pi)/360.0
        self.lidar = RPLidar(port_name)
        self.im_size = img_size
        self.img_draw = np.zeros((img_size[0],img_size[1],3),np.uint8)
        self.Default = self.img_draw
        self.e_angle = error
        self.count = 0 
    def get_data(self):
        try:
            for i,scan in enumerate(self.lidar.iter_scans()):
                t=time.time()
                print('%d: Got %d measurments' % (i, len(scan)))
                self.img_draw = np.zeros((self.im_size[0],self.im_size[1],3),np.uint8)
                for s in scan:
                    x = int(s[2]*self.scale * math.sin((s[1] +self.e_angle)*self.alpha)+ self.Car[0])
                    y = int(s[2]*self.scale * math.cos((s[1] +self.e_angle)*self.alpha)+ self.Car[1])
                    cv2.circle(self.img_draw,(x,y), 2, (255, 255, 255), 1)
                self.img_draw = cv2.rotate(cv2.flip(self.img_draw,0),cv2.ROTATE_90_CLOCKWISE)
                #img_hough = cv2.cvtColor(self.img_draw,cv2.COLOR_BGR2GRAY)
                #self.hough_transform(img_hough)
                cv2.imshow("Lidar",self.img_draw)
                cv2.waitKey(1)
                #print("fps:",1/(time.time()-t))
                #t=time.time()
        except Exception as e:
            print(e)
            pass
        # self.lidar.stop()
        # self.lidar.stop_motor()
        # self.lidar.disconnect()
    def get_center(self,angle,d):
        x = math.sin(angle)*d
        y = math.tan(angle)*x
        return x,y
    def rectangle(self):
        # hrec: chieu cao cua hinh chu nhat
        # wrec: chieu rong cua hinh chu nhat
        start_point = (430 ,0) # x = h/2-hrec/2,y = 0
        end_point = (530, 100) # x = h/2+hrec/2,y = wrec
        color = (255, 128, 0)
        cv2.rectangle(self.img_draw,start_point,end_point,color,1)
    def hough_transform(self,img):
        global black_im1
        lines = cv2.HoughLines(img,1,1*np.pi/180,30)
        black_im = np.zeros((self.im_size[0],self.im_size[1],3),np.uint8)
        if lines is not None:
            for line in lines:
                rho,theta = line[0]
                theta1 = theta*180/np.pi
                cv2.circle(black_im,(int(rho),int(theta1)),5,[255,255,255],5)
        cv2.imshow("Hough_transform",black_im)
        cv2.waitKey(1)
        img_grey=cv2.cvtColor(black_im, cv2.COLOR_RGB2GRAY)
        ret,thresh_img = cv2.threshold(img_grey, 125, 255, cv2.THRESH_BINARY)
        if cv2.getVersionMajor() in [2, 4]:
            # OpenCV 2, OpenCV 4 case
            contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            # OpenCV 3 case
            _, contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        for cnt in contours:
            print("Dien tich:",cv2.contourArea(cnt))
            x,y,w,h = cv2.boundingRect(cnt)
            black_im1 = cv2.rectangle(black_im,(x,y),(x+w,y+h),(255,10,0),2)
        cv2.imshow("Hough_transform",black_im1)
        cv2.waitKey(1)
        # black_im1=None


