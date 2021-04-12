import numpy as np
import matplotlib.pyplot as plt

import math
from src.pathPlanning import spline






print("helloString")
s = spline.generate_cubic_spline(0, 0, math.pi/2, 2, 5, math.pi/6)
print(s)



ax = plt.figure().add_subplot()
# Prepare arrays x, y, z
t_range = np.linspace(0, 1, 1000)

for t in t_range:
    x, y = s.get_pos(t)
    ax.plot(x, y, markersize=3, marker='o')





ax.legend()

plt.show()