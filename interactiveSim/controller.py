
from PID import PID
from car import Car
import numpy as np


# Task 2 controllers

class Precompensator:
    def __init__(self, Kp, Ki, Ts, initial_setpoint_ref):
        self.KP = Kp
        self.Ki = Ki
        self.Ts = Ts

        denom = Kp + Ki * Ts
        self.alpha = (Ki * Ts) / denom
        self.beta = Kp / denom
        self.Vf_prev = initial_setpoint_ref

    def filter(self, Setpoint_ref_current):
        Vf_current = self.alpha * Setpoint_ref_current + self.beta * self.Vf_prev
        self.Vf_prev = Vf_current
        return Vf_current
    
class V_controller:

    def __init__(self, Kp=4323.888, Ki=3647.3125, Ts=1/60, umax=10000, umin=-10000, Kaw=1):
        #Kaw = 1.0/Ts #standard value

        v0 = 27.78
        a = 0.2
        b = 20
        F_roll = 100
        F_drag = F_roll + a * v0**2 + b * v0
        #insantiate controller
        self.PI = PID(Kp=Kp, Ki=Ki, Ts=Ts, umax=umax, umin=umin, Kaw=Kaw, initialState=F_drag)
        self.precomp = Precompensator(Kp=Kp, Ki=Ki, Ts=Ts, initial_setpoint_ref=v0)

    def update(self, desired_speed, speed):
        V_ref_filtered = self.precomp.filter(desired_speed)
        
        force = self.PI.update(V_ref_filtered, speed)
        
        return force


# Task 3 Controllers

#TODO check
class Precompensator_t3:
    def __init__(self, Ts = 1/60, alpha = 1.5):
        self.y_f = 0
        self.alpha = alpha
        self.Ts = Ts

    def filter(self, y):
        
        self.y_f += self.Ts * self.alpha*(y-self.y_f)
        
        return self.y_f
    


class Y_controller:
    #TODO change Kp inner (tested with higher than 0.1240 or whatever it was)
    def __init__(self, Ts=1/60,
                Kp_inner=0.5, delta_max=0.05, delta_min=-0.05,
                # Kp_outer=0.1440, Ki_outer=.1440, phi_max=np.deg2rad(20), phi_min=-np.deg2rad(20), P_loc = 2): 
                phi_max=np.deg2rad(20), phi_min=-np.deg2rad(20), P_loc = 2):
                # Kp_outer=0.2160, Ki_outer=0.3240, phi_max=np.deg2rad(200), phi_min=-np.deg2rad(200)): old
        #Kaw = 1.0/Ts #standard value

        # Caluclate Precomp Values:


        v0 = 27.78

        

        kp = 2*P_loc/v0

        ki = v0*kp^2/4

        #TODO none of this is used?
        # a = 0.2
        # b = 20
        # F_roll = 100
        # F_drag = F_roll + a * v0**2 + b * v0
        #init controller
        self.c_inner = PID(Kp=Kp_inner, Ts=Ts, umax=delta_max, umin=delta_min) # Detla Controller Psi des -> Delta
        self.c_outer = PID(Kp=Kp_outer, Ki=Ki_outer, Ts=Ts, umax=phi_max, umin=phi_min, Kaw=1) #TODO change from 0, initialState=0.0) #Y controller Y-> psi_des

        self.precomp = Precompensator(Kp=Kp_outer, Ki=Ki_outer, Ts=Ts, initial_setpoint_ref=0) # Precomp


    def update(self, y_des, y, phi):

        y_filt = self.precomp.filter(y_des)

        # Precomp Bypass FIXME
        # y_filt = y_des

        phi_des = self.c_outer.update(y_filt, y)

        delta = self.c_inner.update(phi_des, phi)
    
        # print(f"Y = {y}, y_filt = {y_filt}, e_f = {y_filt-y}, phi_des = {phi_des}, phi = {phi}")

        #TODO remove phi_des return
        return delta, phi_des



class Controller:

    # Ts: sample time of the controller;
    # initial_conditions: two element vector representing the equilibrium 
    # i.e., [equilibrium of driving force Fd, equilibrium of vehicle speed]
    def __init__(self, Ts, initial_conditions):
        # parameters go here
        self.Ts = Ts
        self.Fd_cmd = initial_conditions[0]
        self.speed = initial_conditions[1]

        self.F_d_min = -7000.0 #min force (N) (Given)
        self.F_d_max = Car.F_max_force

        # states/controller initialization go here
        self.pos = 0

        self.v_controller = V_controller(Ts = Ts,
                                         umin = self.F_d_min,
                                         umax = self.F_d_max)
        
        self.y_controller = Y_controller()
        



    # speed, y, phi: ego vehicle's speed, lateral position, heading
    # desired_speed: user defined speed setpoint (flexible by plus minus 3 degrees up to speed limits)
    # des_lane: desired lane specified by the user. -1 is left lane, +1 is right lane
    # other_cars: each row of this list corresponds to one of the other vehicles on the road
    #             the columns are: [relative longitudinal position, relative lateral position, relative speed]
    # grade is a road grade object. Use grade.grade_at to find the road grade at any x
    def update(self, speed, x, y, phi, desired_speed, des_lane, other_cars, grade):
        

        Ts = self.Ts
        

        self.desired_y = des_lane * 11.25 # dist from center to lane


        self.vdes = desired_speed

        #TODO remove phi des return
        self.delta_cmd, phi_des = self.y_controller.update(self.desired_y, y, phi)
        self.Fd_cmd = self.v_controller.update(desired_speed, speed) 

        return self.Fd_cmd, self.delta_cmd, phi_des
    
