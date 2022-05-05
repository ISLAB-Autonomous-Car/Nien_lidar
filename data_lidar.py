import csv
import random
from tkinter import N
import numpy as np
import time
import cv2
import pandas as pd
black_im = None
random.seed(time.time())
def fake_data():
    a = -1
    b = 800
    for x in range(200,600,20):
        y = a*x + b + random.random()*6
        x = x + random.random()*6
        print(x,y)
        with open("data_lidar_noise.csv","a",newline="") as f:
            writer = csv.writer(f)
            writer.writerow([x,y])

def hough(data):
    global black_im
    black_im = np.zeros((760,960,3),np.uint8)
    for x,y in data:
        cv2.circle(black_im,(int(x),int(y)),2,(128,128,0),2)
    # img_hough = cv2.cvtColor(black_im,cv2.COLOR_BGR2GRAY)
    # lines = cv2.HoughLinesP(img_hough,10,1*np.pi/180,10)
    # return lines

if __name__ == "__main__":
    file = pd.read_csv("data_lidar_noise.csv")
    file_list = file.values.tolist()
    print(file.values.tolist())
    hough_ = hough(file_list)
    # img_hough = cv2.cvtColor(black_im,cv2.COLOR_BGR2GRAY)
    print(hough_,"Is result")
    if hough_ is not None:
        for i in range(0,len(hough_)):
            l = hough_[i][0]
            cv2.line(black_im, (l[0], l[1]), (l[2], l[3]), (0,0,255),2)
    cv2.imwrite("Hinh3.png",black_im)
    cv2.imshow("Result",black_im)
    cv2.waitKey(0)
    #fake_data()

