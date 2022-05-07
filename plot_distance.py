import pandas as pd
import matplotlib.pyplot as plt

file  = pd.read_csv("distance1.csv",names = ["distance",])

plt.plot(file["distance"]*5,label = "Distance")
plt.xlabel("Frame")
plt.ylabel("Distance(x5mm)")
plt.legend()
plt.show()

