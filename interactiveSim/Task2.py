import numpy as np
import matplotlib.pyplot as plt
import os
from car import car
import control as ct

#from task 1
FIGS_PATH = os.path.abspath("interactiveSim/figs")
fig_ratio = 1.61803398875
fig_height = 4
figsize = (fig_height * fig_ratio, fig_height)


Ts = 1/300
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


v0_100 = 27.78 # KM/H
v0_150 = 150 * v0_100/100 #KM/H




def F_d_ss(v, beta):
    return m*g*np.sin(beta) + Froll + a*v**2 + b*v

def plot_lin_analisis(v0 = 27.78, step_size = 1, sim_length = 150, step_time = 50):

    t = np.arange(0, sim_length + Ts, Ts)
    
    # define plant
    c = 2*a*v0 + b
    plant = ct.tf(1,(m,c))

    # Run Linear Sim
    _, step_res_linaprox = ct.step_response(plant, T = t[int(round(step_time/Ts)):])
    step_res_linaprox = np.concatenate((np.zeros((int(round(step_time/Ts)))), step_res_linaprox))



    # Non lim sim
    eq_input = F_d_ss(v0,0)


    # Create input sequence
    F_d_in = t * 0 + eq_input
    F_d_in[int(round(step_time/Ts)):] = eq_input + step_size

    vel_nonlin_sim_out = np.zeros(len(t))
    nonlin_speed = v0

    for i,_ in enumerate(t):
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
        nonlin_speed += Ts * F_t_nonlin/m
        vel_nonlin_sim_out[i] = nonlin_speed



    plt.figure(figsize=figsize)
    plt.plot(t, vel_nonlin_sim_out)
    plt.plot(t, step_res_linaprox * step_size + v0, "--")

    plt.legend(["Nonlinear", "Linear"])

    plt.title(f"Step Responses: Step Size = {step_size} (N), V0 = {round(v0,2)} (m/s)")

    plt.xlabel("Time (s)")
    plt.ylabel("Delta V (m/s)")

    fig_path = os.path.join(FIGS_PATH, f"t2_Linsim_step{step_size}_v0{round(v0)}.png")
    plt.savefig(fig_path, dpi = 600)

    plt.show()

plot_lin_analisis()

plot_lin_analisis(step_size=600)

plot_lin_analisis(v0 = v0_150)

plot_lin_analisis(v0 = v0_150, step_size=600)