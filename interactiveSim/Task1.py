import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

FIGS_PATH = os.path.abspath("interactiveSim/figs")

Amp = 3


x = np.arange(0, 6101, 1)                     # 6.1 km total 
beta = Amp * np.sin(2*np.pi/1000*x + 300)     # unit is degrees here! 
beta[(x < 500) & (beta < 0)] = 0    # initial 500m is flat

N_e = np.linspace(1000, 5500, 300)
T_e = np.linspace(0, 200, 300)
N, T = np.meshgrid(N_e, T_e)
T_max = 200 - 0.00008 * (N - 3000)**2
T_max = np.clip(T_max, 0, None) 
BSFC = ((N - 2700)/12000)**2 + ((T - 150)/600)**2 + 0.07
BSFC_masked = np.ma.masked_where(T > T_max, BSFC)

fig_ratio = 1.61803398875
fig_height = 4
figsize = (fig_height * fig_ratio, fig_height)

plt.figure(figsize = figsize)
plt.plot(x, beta)
plt.title("Task 1: Default Road Shape")
plt.xlabel("Distance (m)")
plt.ylabel("Height (m)")
fig_path = os.path.join(FIGS_PATH, "road_shape.png")
plt.savefig(fig_path, dpi = 600)
plt.show()


#2D Contour Plot
plt.figure(figsize=figsize)
cont = plt.contour(N, T, BSFC_masked, levels=40, cmap='viridis')
cap = plt.plot(N, T_max, 'r--', label='Torque Limit')
plt.clabel(cont, inline=True, fontsize=8)
plt.xlabel("Speed (rpm)")
plt.ylabel("Torque (Nm)")
plt.title("BSFC Contour Map")
plt.colorbar(cont, label="BSFC")
fig_path = os.path.join(FIGS_PATH, "BSFC_contour.png")
plt.savefig(fig_path, dpi = 600)
plt.show()

#3D Surface Plot
fig = plt.figure(figsize=figsize)
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(T, N, BSFC_masked, cmap='viridis', edgecolor='none')
ax.view_init(elev = 30, azim = 45)
ax.set_xlabel("Torque (Nm)")
ax.set_ylabel("Engine Speed")
ax.set_zlabel("BSFC")
ax.set_title("BSFC Surface Plot")
fig.colorbar(surf, ax=ax, shrink=0.6)


fig_path = os.path.join(FIGS_PATH, "BSFC_3d.png")
plt.savefig(fig_path, dpi = 600)
plt.show()
