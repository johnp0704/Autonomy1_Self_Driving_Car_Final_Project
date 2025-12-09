
from PID import PID
from car import Car
import numpy as np
from FuelOptimizerMPC_controller_JP import mpc_select_v_des

import enum

class States(enum.Enum):
    STRAIGHT = enum.auto()
    MOVING_LEFT = enum.auto()
    MOVING_RIGHT = enum.auto()


class Other_Car:
    def __init__(self, rel_x, rel_speed):
        self.rel_x = rel_x
        self.rel_speed = rel_speed


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
        self.last_filtered_ref = 0.0

    def update(self, desired_speed, speed):
        V_ref_filtered = self.precomp.filter(desired_speed)
        
        force = self.PI.update(V_ref_filtered, speed)
        
        self.last_filtered_ref = V_ref_filtered
        return force


# Task 3 Controllers
#TODO never used??
class Precompensator_t3:
    def __init__(self, Ts = 1/60, alpha = 1.5):
        self.y_f = 0
        self.alpha = alpha
        self.Ts = Ts

    def filter(self, y):
        
        self.y_f += self.Ts * self.alpha*(y-self.y_f)
        
        return self.y_f
    

class Y_controller:
    def __init__(self, Ts=1/60,
                Kp_inner=0.5, delta_max=0.05, delta_min=-0.05,
                phi_max=np.deg2rad(15), phi_min=-np.deg2rad(15), P_loc = 2):

        # Caluclate controller/Precomp Values:
        v0 = 27.78

        Kp_outer = 2*P_loc/v0
        Ki_outer = v0*Kp_outer**2/4

        self.c_inner = PID(Kp=Kp_inner, Ts=Ts, umax=delta_max, umin=delta_min) # Detla Controller Psi des -> Delta
        self.c_outer = PID(Kp=Kp_outer, Ki=Ki_outer, Ts=Ts, umax=phi_max, umin=phi_min, Kaw=1.5) #TODO change from 0, initialState=0.0) #Y controller Y-> psi_des

        self.precomp = Precompensator_t3(alpha=1) #Precompensator(Kp=Kp_outer, Ki=Ki_outer, Ts=Ts, initial_setpoint_ref=0) # Precomp
        



    def update(self, y_des, y, phi):

        y_filt = self.precomp.filter(y_des)

        phi_des = self.c_outer.update(y_filt, y)

        delta = self.c_inner.update(phi_des, phi)
    
        return delta


class Controller:

    # Lane geometry constants
    LANE_MIDPOINT_OFFSET = 11.25   # center of lane in meters

    def __init__(self, Ts, initial_conditions):
        self.Ts = Ts
        self.Fd_cmd = initial_conditions[0]
        self.speed = initial_conditions[1]

        # Force limits
        self.F_d_min = -7000.0
        c = Car(0, 0)
        self.F_d_max = c.F_max_force

        # Instantiate inner controllers
        self.v_controller = V_controller(
            Ts=Ts,
            umin=self.F_d_min,
            umax=self.F_d_max
        )

        self.y_controller = Y_controller()

        # For compatibility with external plotting (not really used now)
        self.last_v_des_opt = 0.0

    def update(self, speed, x, y, phi, desired_speed, des_lane, other_cars, grade):
        """
        Simplified update method:
        - No autonomous lane-change logic
        - No MPC fuel optimization
        - Pure PI speed control + lateral control
        """

        # ------------------------------------------------------------
        # 1. Use desired_speed directly (no MPC alteration)
        # ------------------------------------------------------------
        v_des = desired_speed
        self.last_v_des_opt = v_des  # maintain compatibility

        # ------------------------------------------------------------
        # 2. Convert lane command to desired lateral position
        #    des_lane = +1 (right lane), -1 (left lane)
        # ------------------------------------------------------------
        desired_y = des_lane * self.LANE_MIDPOINT_OFFSET

        # ------------------------------------------------------------
        # 3. Lateral control (Y_controller)
        # ------------------------------------------------------------
        delta_cmd = self.y_controller.update(desired_y, y, phi)

        # ------------------------------------------------------------
        # 4. Longitudinal control (V_controller)
        # ------------------------------------------------------------
        Fd_cmd = self.v_controller.update(v_des, speed)

        return Fd_cmd, delta_cmd
