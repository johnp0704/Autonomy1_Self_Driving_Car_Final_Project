import numpy as np
import matplotlib.pyplot as plt
import os
import car
import control as ct

FIGS_PATH = os.path.abspath("interactiveSim/figs")

#from task 1


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

print(f"C = {(a*v0**2 + b*v0)/v0}")


def F_d_ss(v, beta):
    return m*g*np.sin(beta) + Froll + a*v**2 + b*v

eq_input = F_d_ss(v0, 0)

print(f"F_d_ss = {eq_input}")


Amp = 0
x = np.arange(0, 6101, 1)                     # 6.1 km total 
beta = Amp * np.sin(2*np.pi/1000*x + 300)     # unit is degrees here! 
beta[(x < 500) & (beta < 0)] = 0    # initial 500m is flat

t = np.arange(0, step_size * sim_length +1, 1)



# Create input sequence
step_start_time = 50 #s
F_d_in = t * 0 + eq_input
F_d_in[step_start_time*step_size+1:-1] = eq_input + 1
F_d_in[-1] = eq_input + 1

car1 = car.Car(1/step_size, 27.78)
plt.show()

vel_nonlin_sim_out = np.zeros(len(t))
vel_lin_sim_out = np.zeros(len(t))
for i,val in enumerate(t):
    car1.update(F_d_in[i], 0, 0)
    vel_nonlin_sim_out[i] = car1.speed
    # if i%100 == 0:
    #     print(car1.speed)

plt.plot(t, vel_nonlin_sim_out)
# plt.plot(t,F_d_in)

print()
plt.show()