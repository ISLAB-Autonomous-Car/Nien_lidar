
import numpy as np
import cv2
import math
import math
from rplidar import RPLidar
class cluster_Lidar:
    def __init__(self,port_name,img_size,lamda,error):
        # self.point = point 
        self.x = img_size[0]/2
        self.y = img_size[1]/2
        self.lamda = lamda
        self.error = error
        self.alpha = np.pi/180
        self.lidar = RPLidar(port_name)
        self.im_size = img_size
        self.img_draw = np.zeros((img_size[0],img_size[1],3),np.uint8)
        self.scale = 0.3
    def get_data(self):
        try:
            for i,scan in enumerate(self.lidar.iter_scans()):
                print('%d: Got %d measurments' % (i, len(scan)))
                self.img_draw = np.zeros((self.im_size[0],self.im_size[1],3),np.uint8)
                scan_ = self.cluster_dist(scan)
                #print(scan_)
                # phan cum kieu du lieu la T va R
                try:
                    data_clust = self.data_clust1(scan_)
                    # gom cum kieu du lieu la xy
                except Exception  as e :
                    print(e)
                #print(len(data_clust))
                arr_xy = np.array(data_clust)
                #print(data_xy)
                #print(arr_xy)
                for data in arr_xy:
                    data = np.array(data)
                    #print(data)
                    self.minarearect(data,self.img_draw)
                self.lidar_show(scan_)
                print(" Done ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        except Exception as e:
            print(e)
        #     pass
    # hàm phân cụm 

    def cluster_dist(self,scan):
        lst_scan = []
        for sc in scan:
            lst_scan.append(list(sc))
        #print(lst_scan)
        cls = 0
        for i in range(2,len(lst_scan)):
            # if lst_scan[i][1] > 260 and lst_scan[i][1]  < 280:
            #     print(lst_scan)
            delta = (lst_scan[i][1]- lst_scan[i-1][1])*self.alpha
            if delta < 0:
                delta = delta + 360*self.alpha
            Dmax = abs(lst_scan[i-1][2]*(math.sin(delta)/math.sin(self.lamda-delta))+ 3* self.error) + 3*self.error
            d = math.sqrt((lst_scan[i][2])**2+(lst_scan[i-1][2])**2 - 2*lst_scan[i][2]*lst_scan[i-1][2]*math.cos(delta*self.alpha))
            if d > Dmax:
                print(delta,"la goc lech : 1")
                print("dmax", Dmax, "d", d, "2") 
                #lst_scan[i-1].append(cls)
                cls += 1
                lst_scan[i].append(cls)
            else:
                lst_scan[i].append(cls)
        lst_scan[0].append(0)
        lst_scan[1].append(0)
        lst_scan = self.check_data_clust(lst_scan)
        return lst_scan
    def lidar_show(self,lidar_scan):
        for scan in lidar_scan:
            x = int(scan[2]*math.sin(scan[1]*self.alpha)*self.scale) + 360
            y = int(scan[2]*math.cos(scan[1]*self.alpha)*self.scale) + 480
            #print(x,y,"la toa do")
            if scan[3] is None:
                scan[3] = 0
            np.random.seed(scan[3])
            color = np.random.randint(256, size = 3)
            color = (int(color[0]),int(color[1]),int(color[2]))
            cv2.circle(self.img_draw,(x,y),2,color,1)
        cv2.imshow("Result",self.img_draw)
        cv2.waitKey(1)
    # hàm phân cụm chuyển từ dạng data gốc [ 15, theta, radius, class] về dạng [[class]] với kiểu dữ liệu là x và y
    def data_clust1(self,data):
        data_point = []
        sub_point = []
        index = 0
        while True:
            for point in data:
                if point[3] == index:
                    #chuyen xy trong ham nay luon
                    sub_point.append([int(point[2]*math.sin(point[1]*self.alpha)*self.scale) + 360 ,int(point[2]*math.cos(point[1]*self.alpha)*self.scale) + 480])
            if len(sub_point):
                data_point.append(sub_point)
                index +=1
                sub_point = []
            else:
                break
        #print(data_point,"toa do data clust")
        return data_point
    def check_data_clust(self,data_point):
        first_point = data_point[0]
        end_point = data_point[-1]
        print(data_point,"before coverting")
        if abs(end_point[1] - first_point[1]) < 5:
            for i in range(0,len(data_point)):
                if data_point[i][3] == end_point[3]:
                    data_point[i][3] = first_point[3]
        print(data_point,"After converting")
        return data_point


            
    def minarearect(self,data,im):
        rect = cv2.minAreaRect(data)
        # tọa độ của 4 góc hình chữ nhật ( box )
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        #print("Box......",box,"data",data)
        cv2.drawContours(im,[box],0,(255,0,0),2)    








         
