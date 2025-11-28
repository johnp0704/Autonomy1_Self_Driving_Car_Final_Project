import numpy as np

class car:
    #consts
    m = 1300.0
    F_roll = 100.0 # N
    a = 0.2 # Ns^2/m2
    b = 20.0 # Ns/m
    g = 9.8 # m/s^2
    L = 2.7 # m
    zeta = 0.95
    eta_g = 0.8
    eta_d = 3.8
    rw = 0.34 # m
    Te_max = 200 # Nm
    #end consts

    F_max_force = (Te_max * eta_g * eta_d) / rw * zeta
    F_bar = 200.0
    
    AMP = 0.0
    road_x = np.arange(0, 6101, 1) 
    road_beta_deg = AMP * np.sin(2*np.pi/1000*road_x + 300)
    road_beta = np.deg2rad(road_beta_deg)


    def __init__(self, Ts, initial_speed = 27.78, mass=1300, initial_position=0.0):
        self.Ts = Ts
        
        # Car states
        self.speed = initial_speed #m/s
        self.x = initial_position #longitudinal position (m)
        self.phi = 0.0 #yaw angle (rad)
        self.total_fuel_used_mg = 0.0 #total fuel consumed (mg)
        self.total_distance_m = 0.0 #total distance traveled (m)

        self.fd_min = -7000.0 #min force (N)
        self.delta_max = 0.05 #steering limit (rad)
        self.vx = 0
        self.vy = 0


    def get_road_grade(self):
        x_clamped = np.clip(self.x, self.road_x[0], self.road_x[-1])
        beta_rad = np.interp(x_clamped, self.road_x, self.road_beta)
        return beta_rad

    def update(self, Fd, delta):
        Ts = self.Ts
        
        #saturations
        F_sat = np.clip(Fd, self.fd_min, self.F_max_force)
        delta_sat = np.clip(delta, -self.delta_max, self.delta_max)
        
        #road grade
        beta = self.get_road_grade()
        
        #calculate forces
        F_air = self.a * self.speed**2 + self.b * self.speed
        F_gravity = self.m * self.g * np.sin(beta)
        F_t = F_sat - F_air - self.F_roll - F_gravity
    
        #longitudinal dynamics
        accel = F_t / self.m
        self.speed += accel * Ts
        self.speed = np.maximum(0.0, self.speed) #non-negative speed
        
        self.vx += self.speed * np.cos(self.phi) # speed in direction of road
        self.vy += self.speed * np.sin(self.phi) # Speed left and right

        #position and distance updates
        if self.speed > 0:
            distance_travelled = self.speed * Ts
            self.x += distance_travelled
            self.total_distance_m += distance_travelled
            
        #J_ss calculations        
        N_e = (60/(2*np.pi)) * (self.eta_g * self.eta_d / self.rw) * self.speed
        T_e = (1/self.zeta) * (self.rw / (self.eta_g * self.eta_d)) * F_sat
        BSFC = ((N_e - 2700) / 12000)**2 + ((T_e - 150) / 600)**2 + 0.07
        
        fuel_rate_mg_per_s = np.maximum((1/self.zeta) * BSFC * F_sat * self.speed, self.F_bar)
        
        #update total fuel used
        self.total_fuel_used_mg += fuel_rate_mg_per_s * Ts

        #return values
        return self.speed, self.x, self.total_fuel_used_mg