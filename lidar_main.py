from Lidar_cluster import cluster_Lidar
lidar = cluster_Lidar("COM8",[720,960],10,0.06)
if __name__ == "__main__":
    lidar.get_data()
    
    