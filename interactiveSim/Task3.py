import numpy as np
import matplotlib.pyplot as plt
import os
from car import Car
from controller import Y_controller as controller

v0 = 27.78
L = 2.7
dt = 0.001
T = 2.0
t = np.arange(0, T + dt, dt)

pulse_amp = 0.005
pulse_start = 0.2
pulse_end = 0.4

def steering_delta(time):
    return pulse_amp * (pulse_start <= time < pulse_end)

#nonlinear simulation
car_nl = Car(Ts=dt, initial_speed=v0)

phi_nl = np.zeros_like(t)
y_nl = np.zeros_like(t)
speed_nl = np.zeros_like(t)

for i in range(len(t) - 1):
    delta = steering_delta(t[i])

    #update longitudinal dynamics
    v, _, _ = car_nl.update(Fd=0.0, delta=delta, beta=0)

    speed_nl[i+1] = v

    dphi = (v / L) * np.tan(delta)
    dy = v * np.sin(phi_nl[i])

    phi_nl[i+1] = phi_nl[i] + dphi * dt
    y_nl[i+1] = y_nl[i] + dy * dt

#linearized simulation
phi_lin = np.zeros_like(t)
y_lin = np.zeros_like(t)

for i in range(len(t) - 1):
    delta = steering_delta(t[i])

    dphi = (v0 / L) * delta
    dy = v0 * phi_lin[i]

    phi_lin[i+1] = phi_lin[i] + dphi * dt
    y_lin[i+1] = y_lin[i] + dy * dt


err_phi = phi_nl - phi_lin
err_y   = y_nl  - y_lin

print('Pulse amplitude:', pulse_amp, 'rad')
print('Max yaw error  =', np.max(np.abs(err_phi)))
print('Max lateral error =', np.max(np.abs(err_y)))

#plotting
fig_ratio = 1.61803398875
fig_height = 4
figsize = (fig_height * fig_ratio, fig_height)

#save figures
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FIGS_PATH = os.path.join(SCRIPT_DIR, "figs")

plt.figure(figsize=figsize)
plt.plot(t, phi_nl, label='Nonlinear phi')
plt.plot(t, phi_lin, '--', label='Linear phi')
plt.xlabel('Time (s)')
plt.ylabel('Yaw angle (rad)')
plt.title('Nonlinear vs Linear Yaw Response')
plt.legend()
fig_path = os.path.join(FIGS_PATH, 'Task3_Linearization_Yaw')
plt.savefig(fig_path, dpi = 400)
plt.grid() 
plt.show()

plt.figure(figsize=figsize)
plt.plot(t, y_nl, label='Nonlinear y')
plt.plot(t, y_lin, '--', label='Linear y')
plt.xlabel('Time (s)')
plt.ylabel('Lateral Position y (m)')
plt.title('Nonlinear vs Linear Lateral Response')
plt.legend()
fig_path = os.path.join(FIGS_PATH, 'Task3_Linearization_Position')
plt.savefig(fig_path, dpi = 400)
plt.grid()
plt.show()

plt.figure(figsize=figsize)
plt.plot(t, err_phi, label='Yaw error phi')
plt.plot(t, err_y, label='Lateral error y')
plt.xlabel('Time (s)')
plt.title('Errors')
plt.legend()
fig_path = os.path.join(FIGS_PATH, 'Task3_Linearization_Errors')
plt.savefig(fig_path, dpi = 400)
plt.grid()
plt.show()

# _____________________ SIM _______________________________-
C_inner_Kp = 2

C_outer_Kp = 0.2160
C_outer_Ki = 0.3240

road_grade = 0
y_ref_list = [0.1, 20]

car_lin = Car(Ts=dt, initial_speed=v0)

#car parameters
Tp = 1/300
Ts = 1/60
m = 1300 #kg
Froll = 100 #N
a = 0.2 #Ns^2/m2
b = 20 #Ns/m
g = 9.8 #m/s^2
fd_min = -7000 #N
zeta = 0.95
eta_g = 0.8
eta_d = 3.8
rw = 0.34 #m
F_bar =  200 #m𝑔/s
Te_max = 200 #Nm
F_max = (Te_max * eta_g * eta_d) / rw * zeta #mg/s from engine
F_min = -7000 #mg/s from brakes
L = 2.7 #m
delta_max = 0.05 #rad


v0_100 = 27.78 # KM/H
v0_150 = 150 * v0_100/100 #KM/H

SIM_TIME = 150.0
STEP_TIME = 2


def simulate_step_response(y_start, y_target, plot_title, filename, road_amp=0.0):
    
    # Update car class road profile manually
    Car.AMP = road_amp
    Car.road_beta_deg = Car.AMP * np.sin(2*np.pi/1000*Car.road_x + 300)
    Car.road_beta = np.deg2rad(Car.road_beta_deg)

    #defaults are set appropriately for designed controller of this specific task
    y_controller = controller(Ts=Ts)
    
    # Instantiate car object
    car_task3 = Car(Ts=Ts, initial_speed=v0)

    time_data = np.arange(0, SIM_TIME, Ts)
    y_pos_data = np.zeros_like(time_data)
    ref_data = np.zeros_like(time_data)
    delta_list = np.zeros_like(time_data)
    
    car_task3.y = y_start

    def F_d_ss(v, beta):
        return m*g*np.sin(beta) + Froll + a*v**2 + b*v
    
    # stay at v0 for sim
    control_force =  F_d_ss(v0, 0)

    for i, t in enumerate(time_data):
        desired_y = y_start
        if t >= STEP_TIME:
            desired_y = y_target
        ref_data[i] = desired_y

        delta = y_controller.update(desired_y, car_task3.y, car_task3.phi)
        delta_list[i] = delta

        # Using car class update
        car_task3.update(control_force, delta, 0)
        y_pos_data[i] = car_task3.y
    #plotting

    if filename:

        #Speed vs Time
        plt.figure(figsize=figsize)
        plt.plot(time_data, y_pos_data, label='Actual y Position (m)')
        plt.plot(time_data, ref_data, label='Desired y Posiiton (yref)')
        plt.axvline(STEP_TIME, linestyle=':', label='Step Time')
        plt.title(plot_title)
        plt.xlabel('Time (s)')
        plt.ylabel('y Posiiton (m)')
        plt.xlim([0,8])
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        save_path = os.path.join(FIGS_PATH, filename)
        plt.savefig(save_path, dpi = 400)
        plt.show()

        
for y_ref in y_ref_list:
        simulate_step_response(y_start=0, y_target=y_ref, plot_title=f'Y Step Simulation (y_ref={y_ref})', filename=f"T3_ystep_sim_yref_{int(y_ref)}.png", road_amp=0.0)