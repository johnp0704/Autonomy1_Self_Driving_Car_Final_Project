
from PID import PID
from car import Car
import numpy as np
from controller_task2 import controller_t2 as V_controller




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

        self.desired_y = 0
        self.vdes = desired_speed

        self.delta_cmd = 0
        self.Fd_cmd = self.v_controller.update(desired_speed, speed) 

        return self.Fd_cmd, self.delta_cmd