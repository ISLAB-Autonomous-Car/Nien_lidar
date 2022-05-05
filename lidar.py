from rplidar import RPLidar
import numpy as np
import cv2
import math
import time
lidar = RPLidar('COM8')

info = lidar.get_info()
#print(info)

health = lidar.get_health()
#print(health)
scan=lidar.iter_scans()
de2ra= 0.0174533
bphai = 0.5
btrai = 0.1

scale = 0.3
w = 720
h  = 950
Y = 6
ran_max = 400
alpha = (2*math.pi)/360.0
try:
    for i, scan in enumerate(lidar.iter_scans()):
        t = time.time()
        print('%d: Got %d measurments' % (i, len(scan)))
        #print (scan,"############################################")
        # try:
        image_draw = np.zeros((w, h,3), np.uint8)
        #print(image_draw,"is size of image")
        for scan_ in scan:
            x=int(scan_[2]*scale * math.sin((scan_[1]+Y)*alpha)+w/2)
            y=int(scan_[2]*scale * math.cos((scan_[1]+Y)*alpha)+h/2)
            print(x,y)
            cv2.circle(image_draw, (x, y), 2, (255, 255, 255), -1)
            image_draw_  = cv2.flip(image_draw,0)
        cv2.imshow("lidar",image_draw_)
        cv2.waitKey(1)
except:
    pass
