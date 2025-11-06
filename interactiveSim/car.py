import numpy as np

class Car:
    def __init__(self, Ts, initial_speed, mass=1300):
        # car parameters

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
