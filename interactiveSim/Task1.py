import numpy as np


x = np.arange(0, 6101, 1)                     # 6.1 km total 
beta = Amp * np.sin(2*np.pi/1000*x + 300)     # unit is degrees here! 
beta[(x < 500) & (beta < 0)] = 0    # initial 500m is flat