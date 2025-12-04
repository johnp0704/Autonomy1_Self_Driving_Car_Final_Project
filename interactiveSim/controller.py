
from PID import PID
from car import Car
import numpy as np


# Task 2 controllers

class Precompensator_t2:
    def __init__(self, Kp, Ki, Ts, initial_v_ref):
        self.KP = Kp
        self.Ki = Ki
        self.Ts = Ts

        denom = Kp + Ki * Ts
        self.alpha = (Ki * Ts) / denom
        self.beta = Kp / denom
        self.Vf_prev = initial_v_ref

    def filter(self, V_ref_current):
        Vf_current = self.alpha * V_ref_current + self.beta * self.Vf_prev
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
        self.precomp = Precompensator_t2(Kp=Kp, Ki=Ki, Ts=Ts, initial_v_ref=v0)

    def update(self, desired_speed, speed):
        V_ref_filtered = self.precomp.filter(desired_speed)

        force = self.PI.update(V_ref_filtered, speed)
        return force


# Task 3 Controllers


class Precompensator_t2:
    def __init__(self, Kp, Ki, Ts, initial_v_ref):
        self.KP = Kp
        self.Ki = Ki
        self.Ts = Ts

        denom = Kp + Ki * Ts
        self.alpha = (Ki * Ts) / denom
        self.beta = Kp / denom
        self.Vf_prev = initial_v_ref

    def filter(self, V_ref_current):
        Vf_current = self.alpha * V_ref_current + self.beta * self.Vf_prev
        self.Vf_prev = Vf_current
        return Vf_current
    
class Delta_Inner_controller:

    def __init__(self, Kp=4323.888, Ki=3647.3125, Ts=1/60, umax=10000, umin=-10000, Kaw=1):
        #Kaw = 1.0/Ts #standard value

        v0 = 27.78
        a = 0.2
        b = 20
        F_roll = 100
        F_drag = F_roll + a * v0**2 + b * v0
        #insantiate controller
        self.PI = PID(Kp=Kp, Ki=Ki, Ts=Ts, umax=umax, umin=umin, Kaw=Kaw, initialState=F_drag)
        self.precomp = Precompensator_t2(Kp=Kp, Ki=Ki, Ts=Ts, initial_v_ref=v0)

    def update(self, desired_speed, speed):
        V_ref_filtered = self.precomp.filter(desired_speed)

        force = self.PI.update(V_ref_filtered, speed)
        return force



class Delta_Outer_controller:

    def __init__(self, Kp=4323.888, Ki=3647.3125, Ts=1/60, umax=10000, umin=-10000, Kaw=1):
        #Kaw = 1.0/Ts #standard value

        v0 = 27.78
        a = 0.2
        b = 20
        F_roll = 100
        F_drag = F_roll + a * v0**2 + b * v0
        #insantiate controller
        self.PI = PID(Kp=Kp, Ki=Ki, Ts=Ts, umax=umax, umin=umin, Kaw=Kaw, initialState=F_drag)
        self.precomp = Precompensator_t2(Kp=Kp, Ki=Ki, Ts=Ts, initial_v_ref=v0)

    def update(self, desired_speed, speed):
        V_ref_filtered = self.precomp.filter(desired_speed)

        force = self.PI.update(V_ref_filtered, speed)
        return force


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

        self.delta_cmd = 0
        self.Fd_cmd = self.v_controller.update(desired_speed, speed) 

        return self.Fd_cmd, self.delta_cmd