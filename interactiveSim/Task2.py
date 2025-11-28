import numpy as np
import matplotlib.pyplot as plt
import os
from car import car
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

sim_length = 150 # s
v0_60 = 27.78


def F_d_ss(v, beta):
    return m*g*np.sin(beta) + Froll + a*v**2 + b*v

def plot_lin_analisis(v0 = 27.78, step_size = 300):
    t = np.arange(0, step_size * sim_length +1, 1)

    # Find C and D
    # c = (a*v0**2 + b*v0)/v0
    d = a*v0**2 + b*v0

    c = 2*a*v0 + b

    print(f"c_0 = {c}, d_0 = {d}")

    eq_input = F_d_ss(v0, 0)

    print(f"F_d_ss = {eq_input}")

    # Create input sequence
    step_start_time = 50 #s
    F_d_in = t * 0 + eq_input
    F_d_in[step_start_time*step_size+1:-1] = eq_input + 1
    F_d_in[-1] = eq_input + 1

    car1 = car(1/step_size, 27.78)
    plt.show()

    vel_nonlin_sim_out = np.zeros(len(t))
    vel_lin_sim_out = np.zeros(len(t))

    nonlin_speed = v0
    lin_speed = v0
    for i,val in enumerate(t):
        # car1.update(F_d_in[i], 0, 0)
        # vel_nonlin_sim_out[i] = car1.speed
        # # if i%100 == 0:
        # #     print(car1.speed)0
        F_c = 0 # no grade
        F_sat = F_d_in[i]


        #Non lin sim
        F_air_nonlin = a*nonlin_speed**2 + b*nonlin_speed
        # Sum of forces 
        F_t_nonlin = F_sat - F_air_nonlin - F_c - Froll
        # Longitudinal dynamics (Euler)
        nonlin_speed += 1/step_size * F_t_nonlin/m
        vel_nonlin_sim_out[i] = nonlin_speed


        # Linear sim
        # Forces acting on the car based on speed and road grade
        # F_air = c * lin_speed
        F_air = c * (lin_speed-v0) + d
        # Sum of forces 
        F_t = F_sat - F_air - F_c - Froll

        lin_speed += 1/step_size * F_t/m

        vel_lin_sim_out[i] = lin_speed


    plt.figure()
    plt.plot(t/step_size, vel_nonlin_sim_out-v0)
    plt.plot(t/step_size, vel_lin_sim_out-v0)

    plt.legend(["Nonlinear", "Linear"])

    plt.title(f"Step Responces")

    plt.show()


# plot_lin_analisis(step_size=600)

plot_lin_analisis(step_size=300)