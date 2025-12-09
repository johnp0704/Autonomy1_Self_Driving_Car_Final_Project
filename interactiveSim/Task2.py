import numpy as np
import matplotlib.pyplot as plt
import os
from car import Car
import control as ct

#from task 1
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FIGS_PATH = os.path.join(SCRIPT_DIR, "figs")
fig_ratio = 1.61803398875
fig_height = 4
figsize = (fig_height * fig_ratio, fig_height)


Tp = 1/300
Ts = 1/60
m = 1300 #kg
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

def plot_lin_analisis(v0_sim = 27.78, v0_lin = None, step_size = 1, sim_length = 150, step_time = 50):
    
    # Use v0_sim as the default linearization point if not specified
    if v0_lin is None:
        v0_lin = v0_sim

    t = np.arange(0, sim_length + Ts, Ts)
    
    # Define plant - linearized about v0_lin
    c = 2*a*v0_lin + b
    plant = ct.tf(1,(m,c))

    # Run Linear Sim
    _, step_res_linaprox = ct.step_response(plant, T = t[int(round(step_time/Ts)):])
    step_res_linaprox = np.concatenate((np.zeros((int(round(step_time/Ts)))), step_res_linaprox))

    # Non lim sim
    eq_input = F_d_ss(v0_lin, 0)
    print(f"EQ input (Linearized at v0_lin={round(v0_lin,2)}): {eq_input}N")


    # Create input sequence
    F_d_in = t * 0 + eq_input
    F_d_in[int(round(step_time/Ts)):] = eq_input + step_size

    vel_nonlin_sim_out = np.zeros(len(t))
    nonlin_speed = v0_sim # NONLINEAR SIMULATION STARTS AT v0_sim

    for i,_ in enumerate(t):
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
    plt.plot(t, vel_nonlin_sim_out, label="Nonlinear")
    plt.plot(t, step_res_linaprox * step_size + v0_sim, "--", label="Linear")

    plt.legend()
    plt.title(f"Step Responses: Step Size = {step_size} (N), $v_{{0, \text{{sim}}}} = {round(v0_sim,2)}$ (m/s), $v_{{0, \text{{lin}}}} = {round(v0_lin,2)}$ (m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")

    fig_path = os.path.join(FIGS_PATH, f"t2_Linsim_step{step_size}_v0{round(v0)}.png")
    plt.savefig(fig_path, dpi = 400)

    if __name__ == "__main__":
        plt.show()

plot_lin_analisis(v0_sim = v0_100)
plot_lin_analisis(v0_sim = v0_100, step_size=600)
plot_lin_analisis(v0_sim=v0_100, v0_lin=v0_150)
plot_lin_analisis(v0_sim=v0_100, v0_lin=v0_150, step_size=600)

#==========Controller Simulations============
from controller import V_controller as controller

print(f'\n==========Beginning Simulations=========')

SIM_TIME = 150.0
STEP_TIME = 50.0

V_STEP = 101.0 / 3.6 #m/s

def simulate_step_response(Kp, Ki, Kaw, v_start, v_target, plot_title, filename, road_amp=0.0):
    
    # Update car class road profile manually
    Car.AMP = road_amp
    Car.road_beta_deg = Car.AMP * np.sin(2*np.pi/1000*Car.road_x + 300)
    Car.road_beta = np.deg2rad(Car.road_beta_deg)

    #defaults are set appropriately for designed controller of this specific task
    cruise_controller = controller(Kp=Kp, Ki=Ki, Ts=Ts, umax=F_max, umin=F_min, Kaw=Kaw)
    
    # Instantiate car object
    car_task2 = Car(Ts=Ts, initial_speed=v_start)

    time_data = np.arange(0, SIM_TIME, Ts)
    speed_data = np.zeros_like(time_data)
    ref_data = np.zeros_like(time_data)
    force_data = np.zeros_like(time_data)
    
    current_speed = v_start
    x_pos = 0

    for i, t in enumerate(time_data):
        desired_speed = v_start
        if t >= STEP_TIME:
            desired_speed = v_target
        ref_data[i] = desired_speed

        control_force = cruise_controller.update(desired_speed, current_speed)
        force_data[i] = control_force

        # Using car class update
        grade_at_x = np.interp(x_pos, Car.road_x, Car.road_beta)
        current_speed, x_pos, total_fuel = car_task2.update(control_force, 0, grade_at_x)
        speed_data[i] = current_speed

    #convert back to km/hr
    speed_data_kph = speed_data * 3.6
    ref_data_kph = ref_data * 3.6

    #plotting
    if filename:

        #Speed vs Time
        plt.figure(figsize=figsize)
        plt.plot(time_data, speed_data_kph, label='Actual Speed (v)')
        plt.plot(time_data, ref_data_kph, label='Desired Speed (vref)')
        plt.axvline(STEP_TIME, linestyle=':', label='Step Time')
        plt.title(plot_title)
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (km/hr)')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        save_path = os.path.join(FIGS_PATH, filename)
        plt.savefig(save_path, dpi = 400)
        plt.show()

    return total_fuel
    

#defined gains
Kp = 4323.888
Ki = 3647.3125



if __name__ == "__main__":
    
    #run with no anti_windup, step of 1 km/hr
    simulate_step_response(Kp=Kp, Ki=Ki, Kaw=0.0, v_start=v0_100, v_target=101.0/3.6,
        plot_title='Nonlinear Plant Step Response (1 km/hr step, No Anti-Windup)',
        filename="Task2_Controller_Part3.png",
        road_amp=0
    )

    #rerun with new values
    simulate_step_response(Kp=Kp, Ki=Ki, Kaw=0.0, v_start=v0_100, v_target=150.0/3.6,
        plot_title='Nonlinear Plant Step Response (50 km.hr step, No Anti-Windup)',
        filename="Task2_Controller_Part4.png",
        road_amp=0
    )

    #rerun with anti_windup
    simulate_step_response(Kp=Kp, Ki=Ki, Kaw=1/Ts, v_start=v0_100, v_target=150.0/3.6,
        plot_title='Nonlinear Plant Step Response (50 km.hr step, w/ Anti-Windup)',
        filename="Task2_Controller_Part5.png",
        road_amp=0
    )

    #fuel consumption with road grade
    fuel_full = simulate_step_response(Kp=Kp, Ki=Ki, Kaw=1/Ts, v_start=v0_100, v_target=v0_100, # Constant speed
        plot_title="", filename=None, #no plot needed
        road_amp=3.0)
    print(f"Total Fuel (Full Gains): {fuel_full:.2f} mg")

    #half Gains
    fuel_half = simulate_step_response(Kp=Kp/2, Ki=Ki/2, Kaw=1/Ts, v_start=v0_100, v_target=v0_100,
        plot_title="", filename=None,
        road_amp=3.0)
    print(f"Total Fuel (Half Gains): {fuel_half:.2f} mg")