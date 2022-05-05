import numpy as np
import pandas as pd
from numpy import sqrt
import cv2
import imutils
def read_data(file):
    data_file = pd.read_csv(file)
    data = data_file.values.tolist()
    return data

import random
from numpy import sqrt
def generateRandomIndicesPair(data):
    index1 = random.randint(0, len(data)-1)
    index2 = random.randint(0, len(data)-1)
    while index1 == index2:
        index2 = random.randint(0,len(data)-1)

    return(index1, index2)

def ransac(data):
    numberOfIterations = 100
    allpoints= data
    tolerance = 1
    maximum_inlierscount = 0
    coefficients = []
    for iteration in range(0, numberOfIterations):
        (index1, index2) = generateRandomIndicesPair(data)
        A = allpoints[index1][1] - allpoints[index2][1]
        B = allpoints[index2][0] - allpoints[index1][0]
        C = allpoints[index1][0]*allpoints[index2][1] - allpoints[index1][1]*allpoints[index2][0]
        #equation of line passing through two points is given by (y1 -y2)*X + (x2 - x1)*Y + (x1*y2 - y1*x2) = 0
        #find the count of points lying within tolerance of this line
        inlierscount = 0
        for i in range(0, len(allpoints)):
            #perpendicular distance of point x1, y1 from line Ax + By + C = 0 is |Ax1 + By1 + C|/sqrt(A^2 + B^2)
            distance = abs(A*allpoints[i][0] + B*allpoints[i][1] + C)/(pow(A*A + B*B, 0.5))
            if distance < tolerance:
                inlierscount += 1
        if inlierscount > maximum_inlierscount:
            coefficients = [A, B, C]
            maximum_inlierscount = inlierscount

    # for point in allpoints:
    #     plt.scatter(point[0], point[1], color="green")
        
    A = coefficients[0]
    B = coefficients[1]
    C = coefficients[2]

    # allxvalues = [point[0] for point in allpoints]

    # xvalues = [min(allxvalues), max(allxvalues)]
    # yvalues = [(-C-A*xvalues[0])/B, (-C-A*xvalues[1])/B]
    # xvalues1 =[min(allxvalues), max(allxvalues)]
    # yvalues1 = [(-C-A*xvalues[0])/B-5, (-C-A*xvalues[1])/B-5]
    return A,B,C


def line(x1,y1,x2,y2,data):
    print(type(data))
    black_im = np.zeros((760,960,3),np.uint8)
    cv2.line(black_im,(x1,y1),(x2,y2),(0,128,255),2)
    for x,y in data:
        cv2.circle(black_im,(int(x+300),int(y + 50)), 2, (255, 0, 255), 2)
    black_im = imutils.rotate(black_im, 60)
    cv2.imwrite("Hinh chinh.png",black_im)
    cv2.imshow("Result",black_im)
    cv2.waitKey(0)

# if __name__ =="__main__":
#     data =read_data("data_lidar.csv")
#     a,b,c = ransac(data)
#     print(a,b,c)
#     x1 = data[0][0]
#     y1 = (-c-a*x1)/b
#     x2 = data[-1][0]
#     y2 = (-c-a*x2)/b
#     print(x1,y1,x2,y2)
#     line(int(x1+ 300),int(y1 + 50),int(x2 + 300),int(y2 +50),data)


