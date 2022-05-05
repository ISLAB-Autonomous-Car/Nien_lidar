
import numpy as np
import cv2
import math
import math
from rplidar import RPLidar
import config as cf
from ransac import ransac
cf.turn = None
class cluster_Lidar:
    def __init__(self,port_name,img_size,lamda,error):
        # self.point = point 
        self.x = int(img_size[1]/2)
        self.y = int(img_size[0]/2)

        self.lamda = lamda
        self.error = error
        self.alpha = np.pi/180
        self.lidar = RPLidar(port_name)
        self.im_size = img_size
        self.img_draw = np.zeros((img_size[0],img_size[1],3),np.uint8)
        self.scale = 0.2
        self.carw = 860*self.scale
        self.carh = 860*2*self.scale
        self.side = 800*self.scale
        self.rtrn = 0 
    def get_data(self):
        try:
            for i,scan in enumerate(self.lidar.iter_scans()):
                print('%d: Got %d measurments' % (i, len(scan)))
                self.img_draw = np.zeros((self.im_size[0],self.im_size[1],3),np.uint8)
                scan_ = self.cluster_dist(scan)
                # phan cum kieu du lieu la T va R
                data_clust = self.data_clust1(scan_)
                print(data_clust)
                self.deploy_ransac(data_clust)
                # gom cum kieu du lieu la xy
                self.lidar_show(scan_)
        except Exception as e:
            print(e)
        #     pass
    def cluster_dist(self,scan):
        lst_scan = []
        for sc in scan:
            #self.rtrn = 0
            sc = list(sc)
            if int(sc[2] < 3000): # giới hạn bán kính
                if int(sc[1])>90 and int(sc[1])<180:
                    pass
                else:
                    lst_scan.append(sc)
        cls = 0
        for i in range(2,len(lst_scan)):
            delta = (lst_scan[i][1]- lst_scan[i-1][1])*self.alpha
            if delta < 0:
                delta = delta + 360*self.alpha
            Dmax = abs(lst_scan[i-1][2]*(math.sin(delta)/math.sin(self.lamda-delta))+ 3* self.error) + 3*self.error
            d = math.sqrt((lst_scan[i][2])**2+(lst_scan[i-1][2])**2 - 2*lst_scan[i][2]*lst_scan[i-1][2]*math.cos(delta*self.alpha))
            if d > Dmax:
                cls += 1
                lst_scan[i].append(cls)
            else:
                lst_scan[i].append(cls)
        lst_scan[0].append(0)
        lst_scan[1].append(0)
        lst_scan = self.check_data_clust(lst_scan)
        print(self.rtrn,"is value")
        if self.rtrn > 0:
            print("Ostacle in right side")
            cf.turn = 0
        else:
            print(" Return your lane")
            cf.turn = 1
        self.rtrn = 0
        return lst_scan
    def lidar_show(self,lidar_scan):
        for scan in lidar_scan:
            x = int(scan[2]*math.sin(scan[1]*self.alpha)*self.scale) + self.x
            y = int(scan[2]*math.cos(scan[1]*self.alpha)*self.scale) + self.y
            #
            # self.check_safe(x,y)
            #self.rtrn =0
            if scan[3] is None:
                scan[3] = 0
            np.random.seed(scan[3])
            color = np.random.randint(256, size = 3)
            color = (int(color[0]),int(color[1]),int(color[2]))
            cv2.circle(self.img_draw,(x,y),2,color,1)
            self.axle_car()
        self.img_draw = cv2.flip(self.img_draw,0)
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
                    x = int(point[2]*math.sin(point[1]*self.alpha)*self.scale) + self.x
                    y = int(point[2]*math.cos(point[1]*self.alpha)*self.scale) + self.y
                    if x > 0 and y > 0:
                    #chuyen xy trong ham nay luon
                        sub_point.append([x,y])
            if len(sub_point):
                data_point.append(sub_point)
                index +=1
                sub_point = []
            else:
                break
        return data_point
    def check_data_clust(self,data_point):
        first_point = data_point[0]
        end_point = data_point[-1]
        if abs(end_point[1] - first_point[1]) < 5:
            for i in range(0,len(data_point)):
                if data_point[i][3] == end_point[3]:
                    data_point[i][3] = first_point[3]
        return data_point
    def axle_car(self):
        start_point_y = [self.x,0]
        end_point_y = [self.x,int(self.y*2)]
        start_point_x = [0,self.y]
        end_point_x = [int(self.x*2),self.y]
        cv2.line(self.img_draw,start_point_y,end_point_y,(255,0,0),1)
        cv2.line(self.img_draw,start_point_x,end_point_x,(255,0,0),1)
        start_point_rec = [int(self.x+self.carh),int(self.y-self.carw)]
        end_point_rec = [self.x,self.y]
        cv2.rectangle(self.img_draw,start_point_rec,end_point_rec,(255,196,12),4)
        #safe_start = []
    def check_safe(self,x,y):
        start = [self.x,int(self.y + self.side)]
        end = [int(self.x + self.carh*0.9),self.y]
        cv2.rectangle(self.img_draw,start,end,(0,196,12),1)
        cv2.circle(self.img_draw,tuple(start),2,(0,0,128),5)
        cv2.circle(self.img_draw,tuple(end),2,(128,0,0),5)
        if x > start[0] and  x < end[0] and y < start[1] and y > end[1]:# co point trong box
            #print("have ostacle")
            self.rtrn += 1
        else:
            self.rtrn += 0

    def check_return(self,scan):
        x = int(scan[2]*math.sin(scan[1]*self.alpha)*self.scale) + self.x
        y = int(scan[2]*math.cos(scan[1]*self.alpha)*self.scale) + self.y
        self.rtrn += self.check_safe(x,y)
    def deploy_ransac(self,data_clusted):
        len_max = 0
        if data_clusted is not None:
            for i in range(len(data_clusted)):
                if len(data_clusted[i])>len_max:
                    len_max = i
            #print(len(data_clusted[len_max]), "Is max")
            a,b,c = ransac(data_clusted[len_max])
            print(data_clusted[len_max][0],data_clusted[len_max][-1])
            x1 = int(data_clusted[len_max][0][0]) + self.x
            x2 = int(data_clusted[len_max][-1][0]) + self.x
            y1 = int((c-a*x1)/b) + self.y
            y2 = int((c-a*x2)/b) + self.y
            cv2.line(self.img_draw,(x1,y1),(x2,y2),(0,128,200),2)
        else:
            print("No point in cluster")
            
            


        

            
        

        









         
