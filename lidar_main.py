from Lidar_cluster import cluster_Lidar
from sender import send_sig
import config as cf
import threading
#lidar = cluster_Lidar("/dev/ttyUSB0",[720,960],10,0.03)
lidar = cluster_Lidar("COM8",[820,760],10,0.03)
if __name__ == "__main__":
    # thr1 = threading.Thread(target=lidar.get_data)
    # thr1.start()
    # thr2 = threading.Thread(target= send_sig, args=(cf.turn,"192.168.1.2"))
    # thr2.start()
    lidar.get_data()
    