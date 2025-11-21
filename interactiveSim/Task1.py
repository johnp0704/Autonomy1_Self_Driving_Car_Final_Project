import numpy as np
import matplotlib.pyplot as plt

Amp = 3


x = np.arange(0, 6101, 1)                     # 6.1 km total 
beta = Amp * np.sin(2*np.pi/1000*x + 300)     # unit is degrees here! 
beta[(x < 500) & (beta < 0)] = 0    # initial 500m is flat

fig_ratio = 1.61803398875
fig_height = 4
figsize = (fig_height, fig_height * fig_ratio)

plt.figure(figsize = figsize)
plt.plot(x, beta)
plt.title("Task 1: Default Road Shape")
plt.xlabel("Distance")
plt.ylabel("Height")
plt.savefig()
plt.show()
