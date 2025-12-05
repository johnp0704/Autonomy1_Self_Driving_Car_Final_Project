import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

#Consts
m = 1300
Froll = 100 #𝑁
a = 0.2 #𝑁𝑠^2/𝑚2
b = 20 #𝑁𝑠/𝑚
g = 9.8 #𝑚/𝑠^2
fd_min = -7000 #𝑁
zeta = 0.95
eta_g = 0.8
eta_d = 3.8
rw = 0.34 #𝑚
F_bar =  200 #𝑚𝑔/𝑠
Te_max = 200 #Nm
F_max = (Te_max * eta_g * eta_d) / rw * zeta #mg/s from engine
F_min = -7000 #mg/s from brakes
L = 2.7 #m
delta_max = 0.05 #rad
step_size = 300 #steps per second
sim_length = 150 # s

v0 = 27.78
#End Consts

#save figures
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FIGS_PATH = os.path.join(SCRIPT_DIR, "figs")

Amp = 3

x = np.arange(0, 6101, 1)                     # 6.1 km total 
beta = Amp * np.sin(2*np.pi/1000*x + 300)     # unit is degrees here! 
beta[(x < 500) & (beta < 0)] = 0    # initial 500m is flat

#plotting variables
N_e = np.linspace(1000, 5500, 300)
T_e = np.linspace(0, 225, 300)
N, T = np.meshgrid(N_e, T_e)

#piecewise T_max function
def tmax(N):
    return np.where(N < 3250, 0.1 * N + 50, -0.1 * N + 700)

#given functions
T_max = tmax(N)
T_max = np.clip(T_max, 0, 200)
BSFC = ((N - 2700)/12000)**2 + ((T - 150)/600)**2 + 0.07
BSFC_masked = np.ma.masked_where(T > T_max, BSFC)

fig_ratio = 1.61803398875
fig_height = 4
figsize = (fig_height * fig_ratio, fig_height)

#Road profile plot
plt.figure(figsize = figsize)
plt.plot(x, beta)
plt.title("Task 1: Default Road Shape")
plt.xlabel("Distance (m)")
plt.ylabel("Height (m)")
fig_path = os.path.join(FIGS_PATH, "road_shape.png")
plt.savefig(fig_path, dpi = 600)
plt.grid()
if __name__ == "__main__":
    plt.show()


#2D Contour Plot
plt.figure(figsize=figsize)
cont = plt.contour(N, T, BSFC_masked, levels=40, cmap='viridis')
plt.plot(N_e, T_max[0, :], 'r', linewidth=2, label='Torque Limit')
plt.clabel(cont, inline=True, fontsize=8)
plt.xlabel("Speed (rpm)")
plt.ylabel("Torque (Nm)")
plt.title("BSFC Contour Map")
plt.colorbar(cont, label="BSFC")
fig_path = os.path.join(FIGS_PATH, "BSFC_contour.png")
plt.savefig(fig_path, dpi = 600)
plt.grid()
if __name__ == "__main__":
    plt.show()

#3D Surface Plot
fig = plt.figure(figsize=figsize)
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(T, N, BSFC_masked, cmap='viridis', edgecolor='none')
ax.view_init(elev = 30, azim = 30)
ax.set_xlabel("Torque (Nm)")
ax.set_ylabel("Engine Speed")
ax.set_zlabel("BSFC")
ax.set_title("BSFC Surface Plot")
fig.colorbar(surf, ax=ax, shrink=0.6)
fig_path = os.path.join(FIGS_PATH, "BSFC_3d.png")
plt.savefig(fig_path, dpi = 600)
if __name__ == "__main__":
    plt.show()

#Fdss derivation
def F_d_ss(v, beta):
    return m*g*np.sin(beta) + Froll + a*v**2 + b*v

print(f"F_d_ss = {F_d_ss(v0, 0)} N")

#Jss derivation
def J_ss(v, beta):
    F_d = F_d_ss(v, beta)
    N = (60/(2*np.pi)) * (eta_g * eta_d / rw) * v
    T = (1/zeta) * (rw / (eta_g * eta_d)) * F_d
    BSFC = ((N - 2700)/12000)**2 + ((T - 150)/600)**2 + 0.07
    return np.maximum((1/zeta)*(BSFC) * F_d * v, F_max)

#Jss surface plot
v = np.linspace(23, 28, 1000)
beta = np.linspace(np.deg2rad(-2.5), np.deg2rad(2.5), 1000)

V, B = np.meshgrid(v, beta)
Jss_vals = J_ss(V, B)

#Jss Surface Plot
fig = plt.figure(figsize=figsize)
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(V, B, Jss_vals, cmap='viridis', edgecolor='none')
ax.view_init(elev = 30, azim = 30)
ax.set_xlabel("v")
ax.set_ylabel("beta")
ax.set_zlabel("J_ss")
ax.set_title("J_ss Surface Plot")
fig.colorbar(surf, ax=ax, shrink=0.6)
fig_path = os.path.join(FIGS_PATH, "Jss_3d.png")
plt.savefig(fig_path, dpi = 600)
if __name__ == "__main__":
    plt.show()


#============Validation of model===============
from car import Car
mpg_conversion = 1761.59 #calculated by hand

def simulate(car_obj, Fd_func, duration=150, grade = None, x = None):
    #time vector
    t_vec = np.arange(0, duration, car_obj.Ts)

    #logging lists
    v_log = []
    mpg_log = []
    t_log = []
    x_pos = 0

    for t in t_vec:
        Fd_input = Fd_func(t)
        grade_at_x = 0
        # Interp for grade at x
        if(x is not None):
            grade_at_x = np.interp(x_pos, x, grade)
            
        speed, x_pos, fuel = car_obj.update(Fd_input, 0, grade_at_x)


        if len(mpg_log) > 0: #skip if on first test
            pass

        F_max = car_obj.F_max_force
        F_sat_log = np.clip(Fd_input, car_obj.fd_min, car_obj.F_max_force)

        Ne_log = (60/(2*np.pi)) * (car_obj.eta_g * car_obj.eta_d / car_obj.rw) * speed
        Te_log = (1/car_obj.zeta) * (car_obj.rw / (car_obj.eta_g * car_obj.eta_d)) * F_sat_log
        BSFC_log = ((Ne_log - 2700) / 12000)**2 + ((Te_log - 150) / 600)**2 + 0.07
        fuel_rate_log = np.maximum((1/car_obj.zeta) * BSFC_log * F_sat_log * speed, car_obj.F_bar)

        if fuel_rate_log > 0 and speed > 0.1:
            inst_mpg = (speed / fuel_rate_log) * mpg_conversion
        else:
            inst_mpg = 0
        
        v_log.append(speed)
        mpg_log.append(inst_mpg)
        t_log.append(t)
    return t_log, v_log, mpg_log

#====Sim 1, flat road=====
car_1 = Car(Ts=0.1/step_size, initial_speed=27.78)

flat_beta = np.zeros_like(x)
car_1.road_beta = np.deg2rad(flat_beta)

F_eq_val = F_d_ss(27.78, 0.0)

def step_force(t):
    if t < 50:
        return F_eq_val
    else:
        return 3000.0
    
t1, v1, mpg1 = simulate(car_1, step_force)

#plot
plt.figure(figsize=figsize)
plt.subplot(2, 1, 1)
plt.plot(t1, v1, 'b', linewidth=2, label='v')
plt.ylabel('Velocity (m/s)')
plt.title('Simulation 1: Step Input on Flat Road')
plt.grid()
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t1, mpg1, 'b', linewidth=2, label='mpg')
plt.ylabel('MPG')
plt.xlabel('Time (s)')
plt.grid()
plt.legend()

fig_path = os.path.join(FIGS_PATH, "Validation_Sim_1_Flat_Road.png")
plt.savefig(fig_path, dpi = 600)

if __name__ == "__main__":
    plt.show()

#====Sim 2, Amp=3=====
car_2 = Car(Ts=1.0/step_size, initial_speed=27.78)

sim_x = np.arange(0, 6101, 1)
sim_beta_deg = Amp * np.sin(2*np.pi/1000*sim_x + 300)
sim_beta_deg[(sim_x < 500) & (sim_beta_deg < 0)] = 0 #flatten
sim_beta_rad = np.deg2rad(sim_beta_deg)

# car_2.road_x = sim_x #match axis
# car_2.road_beta = sim_beta_rad

def const_force(t):
    return F_eq_val

t2, v2, mpg2 = simulate(car_2, const_force, grade=sim_beta_rad, x = sim_x)

#plot
plt.figure(figsize=figsize)
plt.subplot(2, 1, 1)
plt.plot(t2, v2, 'b', linewidth=2, label='v')
plt.ylabel('Velocity (m/s)')
plt.title('Simulation 2: Const Force Input on Hilly Road')
plt.grid()
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t2, mpg2, 'b', linewidth=2, label='mpg')
plt.ylabel('MPG')
plt.xlabel('Time (s)')
plt.grid()
plt.legend()

fig_path = os.path.join(FIGS_PATH, "Validation_Sim_2_Hilly_Road.png")
plt.savefig(fig_path, dpi = 600)
if __name__ == "__main__":
    plt.show()