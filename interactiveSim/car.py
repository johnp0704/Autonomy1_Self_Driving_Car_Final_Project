import numpy as np

class Car:
    def __init__(self, Ts, initial_speed, mass=1300):
        # car parameters
        Ts = Ts
        speed = initial_speed #m/s
        m = mass #𝐾𝑔
        Froll = 100 #𝑁
        a = 0.2 #𝑁𝑠^2/𝑚2
        b = 20 #𝑁𝑠/𝑚
        g = 9.8 #𝑚/𝑠^2
        fd_min = -7000 #𝑁
        zeta = 0.95
        eta_g = 0.8
        eta_d = 3.8
        rw = 0.34 #𝑚
        F =  200 #𝑚𝑔/𝑠
        Te_max = 200 #Nm
        F_max = (Te_max * eta_g * eta_d) / rw * zeta #mg/s from engine (calculated)
        F_min = -7000 #mg/s from brakes
        L = 2.7 #m
        delta_max = 0.05 #rad
        step_size = 300 #steps per second

        N_e = (60/(2*np.pi)) * (eta_g * eta_d * speed) / rw

        # car states 
        T_e = 0 #Nm
        # outputs

    def update(self, F, delta, beta):
        # Saturate force and steering
        self.F_sat = np.clip(F, self.F_min, self.F_max)
        self.delta_sat = np.clip(delta, -self.delta_max, self.delta_max)

        # Longitudinal dynamics (Euler)
        self.speed += self.speed * self.Ts
    
        # Yaw kinematics (bicycle model)
        self.phi += self.speed / self.L * np.tan(self.delta_sat) * self.Ts

        # Velocity components in world coordinates (m/s)
        self.vx += self.speed * np.cos(self.phi)
        self.vy += self.speed * np.sin(self.phi)

        # Position update
        self.x += self.vx * self.Ts
        self.y += self.vy * self.Ts

        # Simple fuel model (unchanged)
        self.BSFC = ((self.N_e - 2700) / 12000)**2 + ((self.T_e - 150) / 600)**2 + 0.07
        self.fuel_rate = 
        self.total_fuel = 
