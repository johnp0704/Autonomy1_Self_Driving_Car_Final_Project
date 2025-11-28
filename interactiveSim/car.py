import numpy as np

class Car:
    def __init__(self, Ts = 1/300, initial_speed = 27.78, mass=1300):
        # car parameters
        self.Ts = Ts
        self.speed = initial_speed #m/s
        self.m = mass #𝐾𝑔
        self.F_roll = 100 #𝑁
        self.a = 0.2 #𝑁𝑠^2/𝑚2
        self.b = 20 #𝑁𝑠/𝑚
        self.g = 9.8 #𝑚/𝑠^2
        self.fd_min = -7000 #𝑁
        self.zeta = 0.95
        self.eta_g = 0.8
        self.eta_d = 3.8
        self.rw = 0.34 #𝑚
        self.F_bar =  200 #𝑚𝑔/𝑠
        self.Te_max = 200 #Nm
        self.F_max = (self.Te_max * self.eta_g * self.eta_d) / self.rw * self.zeta #mg/s from engine
        self.F_min = -7000 #mg/s from brakes
        self.L = 2.7 #m
        self.delta_max = 0.05 #rad
        self.step_size = 300 #steps per second

        #Car state vars
        self.N_e = (60/(2*np.pi)) * (self.eta_g * self.eta_d * self.speed) / self.rw
        self.phi = 0
        self.vx = 0
        self.vy = 0
        self.x = self.speed #assume all speed starting out is in the x direction (ie straight)
        self.y = 0
        self.total_fuel_used = 0

        # car states 
        self.T_e = 0 #Nm

        # outputs

    def update(self, F, delta, beta):
        # Saturate force and steering
        F_sat = np.clip(F, self.F_min, self.F_max)
        delta_sat = np.clip(delta, -self.delta_max, self.delta_max)


        # Forces acting on the car based on speed and road grade
        F_air = self.a*self.speed**2 + self.b*self.speed
        F_c = self.m * self.g * np.sin(beta)

        # Sum of forces 
        F_t = F_sat - F_air - F_c - self.F_roll
    
        # Yaw kinematics (bicycle model)
        self.phi += (self.speed / self.L) * np.tan(delta_sat) * self.Ts

        # Longitudinal dynamics (Euler)
        self.speed += self.Ts * F_t/self.m
        # Velocity components in world coordinates (m/s)
        self.vx += self.speed * np.cos(self.phi) # speed in direction of road
        self.vy += self.speed * np.sin(self.phi) # Speed left and right

        


        # Position update
        self.x += self.vx * self.Ts
        self.y += self.vy * self.Ts



        # Simple fuel model (unchanged)
        self.BSFC = ((self.N_e - 2700) / 12000)**2 + ((self.T_e - 150) / 600)**2 + 0.07
        self.fuel_rate = max(1/self.zeta * self.BSFC * F * self.speed, self.F_bar)
        self.total_fuel_used += self.fuel_rate * self.Ts
