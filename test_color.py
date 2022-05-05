import numpy as np
np.random.seed(3)
color = np.random.randint(256, size = 3)
color = tuple(color)
print(type(color[0]))