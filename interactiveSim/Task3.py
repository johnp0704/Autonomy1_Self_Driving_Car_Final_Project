import numpy as np
import matplotlib.pyplot as plt
import os
from car import car

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
car_nl = car(Ts=dt, initial_speed=v0)

phi_nl = np.zeros_like(t)
y_nl = np.zeros_like(t)
speed_nl = np.zeros_like(t)

for i in range(len(t) - 1):
    delta = steering_delta(t[i])

    #update longitudinal dynamics
    v, _, _ = car_nl.update(Fd=0.0, delta=delta)

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
FIGS_PATH = os.path.abspath('interactiveSim/figs')

plt.figure(figsize=figsize)
plt.plot(t, phi_nl, label='Nonlinear phi')
plt.plot(t, phi_lin, '--', label='Linear phi')
plt.xlabel('Time (s)')
plt.ylabel('Yaw angle (rad)')
plt.title('Nonlinear vs Linear Yaw Response')
plt.legend()
fig_path = os.path.join(FIGS_PATH, 'Task3_Linearization_Yaw')
plt.savefig(fig_path, dpi = 600)
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
plt.savefig(fig_path, dpi = 600)
plt.grid()
plt.show()

plt.figure(figsize=figsize)
plt.plot(t, err_phi, label='Yaw error phi')
plt.plot(t, err_y, label='Lateral error y')
plt.xlabel('Time (s)')
plt.title('Errors')
plt.legend()
fig_path = os.path.join(FIGS_PATH, 'Task3_Linearization_Errors')
plt.savefig(fig_path, dpi = 600)
plt.grid()
plt.show()
