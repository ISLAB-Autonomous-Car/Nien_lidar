import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

def get_data():
    lidar = RPLidar('COM8', baudrate=115200)
    for scan in lidar.iter_scans(max_buf_meas=500):
        #print(scan,"##########################")
        break
    lidar.stop()
    return scan

for i in range(1000000):
    if(i%7==0):
        x = []
        y = []
    print(i)
    current_data=get_data()
    print(current_data,"##################")
    for point in current_data:
        if point[0]==15:
            x.append(point[2]*np.sin(point[1]))
            y.append(point[2]*np.cos(point[1]))
    plt.clf()
    plt.scatter(x, y)
    plt.pause(.1)
plt.show()