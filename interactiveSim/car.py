import numpy as np

class Car:
    def __init__(self, Ts, initial_speed, mass=1300):
        # car parameters
        m = 1300 #𝐾𝑔
        Froll = 100 #𝑁
        a = 0.2 #𝑁𝑠^2/𝑚2
        b = 20 #𝑁𝑠/𝑚
        g = 9.8 #𝑚/𝑠^2
        fd_min = -7000 #𝑁
        zeta = 0.95
        eta_g = 0.8
        eta_g = 3.8
        rw = 0.34 #𝑚
        F_ =  200 #𝑚𝑔/𝑠
        L = 2.7 #m
        delta_max = 0.05 #rad


        # car states 

        # outputs

    def update(self, F, delta, beta):
        # Saturate force and steering
        self.F_sat = 
        self.delta_sat = 

        # Longitudinal dynamics (Euler)
        self.speed = 
        
        # Yaw kinematics (bicycle model)
        self.phi =

        # Velocity components in world coordinates (m/s)
        self.vx = 
        self.vy = 

        # Position update
        self.x = 
        self.y = 

        # Simple fuel model (unchanged)
        self.BSFC = 
        self.fuel_rate = 
        self.total_fuel = 
