
from PID import PID
from car import Car
import numpy as np
import math

import enum
#THE BEGINNING OF THIS IS THE SAME AS TASK 2 AND 3 CONTROLLERS
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


#Task 4: putting it all together

#=============BEGIN MPC=================================
#vehicle parameters
m = 1300.0
Froll = 100.0
a = 0.2
b = 20.0
g = 9.8
zeta = 0.95
eta_g = 0.8
eta_d = 3.8
rw = 0.34
F_bar = 200.0  # mg/s minimum fuel
v_min = 20.83
v_max = 27.78

# MPC parameters
Ts_mpc = 1.0
N = 10 #horizon length
tau_v = 1.0 #closed loop speed
lambda_tracking = 1.0

# BSFC function
def bsfc_from_Ne_Te(Ne, Te):
    return ((Ne - 2700.0)/12000.0)**2 + ((Te - 150.0)/600.0)**2 + 0.07

def F_d_ss(v, beta):
    return m*g*np.sin(beta) + Froll + a*v**2 + b*v

def fuel_rate_ss(v, beta):
    Fd = F_d_ss(v, beta)
    # engine torque and speed
    Te = (1.0/zeta) * (rw / (eta_g * eta_d)) * Fd
    Ne = (60.0/(2.0*math.pi)) * (eta_g*eta_d / rw) * v
    bsfc = bsfc_from_Ne_Te(Ne, Te)
    fr = (1.0/zeta) * bsfc * Fd * v
    return max(fr, F_bar)

def predict_other_positions_global(x0, other_cars, k, Ts, v0):
    #Predict global positions of other cars after k steps.
    preds = []
    for oc in other_cars:
        other_speed = v0 + oc[2]
        other_x_k = (x0 + oc[0]) + other_speed * (k * Ts)
        preds.append(other_x_k)
    return preds

def mpc_cost_for_vdes(v_des_candidate, x0, v0, grade_obj, other_cars, v_driver, Ts=Ts_mpc, N=N, tau=tau_v, safety_gap_min=10.0):
    #simulate coarse dynamics forward for constant v_des_candidate and return total cost.
    x = x0
    v = v0
    total_cost = 0.0

    #if any current other car is already closer than safety gap, reject candidates that reduce gap
    for oc in other_cars:
        if oc[0] < safety_gap_min:
            pass

    for k in range(N):
        #road grade at x
        beta_deg = grade_obj.grade_at(x)
        beta = math.atan(beta_deg/100.0)

        #fuel burn at current v
        Jss = fuel_rate_ss(v, beta)
        total_cost += Jss * Ts

        #first-order to v_des
        v = v + (Ts/tau) * (v_des_candidate - v)
        x = x + Ts * v  #predicted ego position at next step

        #predict other vehicles' positions at future time
        for oc in other_cars:
            other_speed = v0 + oc[2]
            other_x_k = (x0 + oc[0]) + other_speed * ((k+1)*Ts)
            ego_x_k = x
            gap = other_x_k - ego_x_k

            #if other vehicle is behind, skip
            if gap <= 0:
                continue

            #if gap ever drops below safety threshold, reject
            if gap < safety_gap_min:
                return 1e9 #huge penalty, candidate not safe

    total_cost += lambda_tracking * (v_des_candidate - v_driver)**2
    return total_cost

def mpc_select_v_des(x0, v0, grade_obj, other_cars, v_driver, search_resolution=0.1, safety_gap_min=16.0):
    # allowable slack: clamp (v_driver +/- 3) into [v_min, v_max]
    low = max(v_min, v_driver - 3.0)
    high = min(v_max, v_driver + 3.0)
    candidates = np.arange(low, high + 1e-6, search_resolution)
    best_v = v_driver
    best_cost = 1e12
    for cand in candidates:
        c = mpc_cost_for_vdes(cand, x0, v0, grade_obj, other_cars, v_driver, Ts=Ts_mpc, N=N, tau=tau_v, safety_gap_min=safety_gap_min)
        if c < best_cost:
            best_cost = c
            best_v = cand
    return best_v

#================END MPC=================================================

class Controller:

    LANE_MIDPOINT_OFFSET = 11.25
    LANE_THRESH = LANE_MIDPOINT_OFFSET*0.9
    LANE_DIFF_LATERAL_READING = 45 # The reading when a car is in another lane
    LAT_READING_CONVERSION_FACTOR = (2*LANE_MIDPOINT_OFFSET)/LANE_DIFF_LATERAL_READING # Conversion factor for other car rel y to meters

    # Safety params
    FOLLOWING_TIME_S = 6.0   # seconds headway to consider for passing
    MIN_ABS_GAP_M = 12.0     # absolute min gap (m) that triggers immediate evasive lane change if possible
    SAFETY_MIN_GAP_PRED = 16.0 # safety gap used for MPC predictions (kept in sync with MPC settings)


        # Ts: sample time of the controller;
    # initial_conditions: two element vector representing the equilibrium 
    # i.e., [equilibrium of driving force Fd, equilibrium of vehicle speed]
    def __init__(self, Ts, initial_conditions):
        # parameters go here
        self.Ts = Ts
        self.Fd_cmd = initial_conditions[0]
        self.speed = initial_conditions[1]

        self.F_d_min = -7000.0 #min force (N) (Given)
        c = Car(0,0)
        self.F_d_max = c.F_max_force

        # states/controller initialization go here
        self.pos = 0

        self.v_controller = V_controller(Ts = Ts,
                                         umin = self.F_d_min,
                                         umax = self.F_d_max)
        
        self.y_controller = Y_controller()

        self.state = States.MOVING_RIGHT
        self.last_v_des_opt = 0.0

        

    def time_to_impact(self, rel_x, rel_speed):
        # rel_x: positive if other car is ahead
        # rel_speed = other_speed - ego_speed
        # We only care if we are closing (rel_speed < 0), i.e., other is slower than us
        if rel_speed >= -0.01:  # not closing (or nearly zero)
            return None

        # avoid divide-by-zero and extremely large times due to tiny rel_speed
        safe_rel_speed = abs(rel_speed)
        if safe_rel_speed < 0.1:
            # treat as "not imminently closing" (will rely on absolute gap)
            return None

        return abs(rel_x / rel_speed)
    

    def state_machine(self, y, other_cars, desired_speed, speed):
        FOLLOWING_DISTANCE = self.FOLLOWING_TIME_S # sec
        MIN_VERT_FOR_LANE_CHANGE = 11 # meters
        
        #init return, desired_speed defaults to input value
        desired_y = None
        anticipated_lane_right = None

        # -------------------------------------- Check if car needs to finish lane change --------------------------
        # Check if a previously started lane change was completed if not complete that turn first
        if (((self.state == States.MOVING_RIGHT) and (y > self.LANE_THRESH))       # Arrived in right lane
            or ((self.state == States.MOVING_LEFT) and (y < -self.LANE_THRESH))):  # Arrived in left lane
            
            # Set to keep no longer turning
            self.state = States.STRAIGHT
        elif((self.state == States.MOVING_RIGHT) or (self.state == States.MOVING_LEFT)):
            if (self.state == States.MOVING_LEFT):
                desired_y = -self.LANE_MIDPOINT_OFFSET
            elif (self.state == States.MOVING_RIGHT): 
                desired_y = self.LANE_MIDPOINT_OFFSET
            else:
                print("ERROR finishing turn")

            return desired_y, desired_speed


        # Determine which lane we are anticipating being in for planning
        if (self.state == States.MOVING_RIGHT) or ((y >= self.LANE_THRESH)
                                                   and (self.state == States.STRAIGHT)):
            anticipated_lane_right = True
        elif (self.state == States.MOVING_LEFT) or ((y <= -self.LANE_THRESH)
                                                   and (self.state == States.STRAIGHT)):
            anticipated_lane_right = False
        else:
            # fallback
            anticipated_lane_right = (y >= 0)




        #----------------------------------------- Gather info on other cars -------------------------------
        # each index: (rel pos, rel speed, time to impact)
        cars_right_lane = []
        cars_left_lane = []

        # Read in all car data
        for npc_car in other_cars:
            
            rel_x = npc_car[0]

            # If car is behind us by enough of a margin we don't care
            if rel_x < -MIN_VERT_FOR_LANE_CHANGE:
                continue

            rel_y = npc_car[1] * self.LAT_READING_CONVERSION_FACTOR
            rel_speed = npc_car[2]  # this is other.speed - ego.speed
            abs_y = y + rel_y

            # Only consider cars that are in approximate lane center (left/right)
            if abs_y > self.LANE_THRESH:
                # In right lane
                tti = self.time_to_impact(rel_x, rel_speed)
                cars_right_lane.append((rel_x, rel_speed, tti))

            elif abs_y < -self.LANE_THRESH:
                # In left lane
                tti = self.time_to_impact(rel_x, rel_speed)
                cars_left_lane.append((rel_x, rel_speed, tti))

            else:
                # treat as in same lane - consider for immediate safety
                tti = self.time_to_impact(rel_x, rel_speed)
                # place it in the closer lane list (based on sign of rel_y)
                if rel_y >= 0:
                    cars_right_lane.append((rel_x, rel_speed, tti))
                else:
                    cars_left_lane.append((rel_x, rel_speed, tti))
            
            # print(f"Car Y: {y:<6.2f}| Rel x: {npc_car[0]:<6.2f}| Rel y: {npc_car[1]* self.LAT_READING_CONVERSION_FACTOR:<6.2f}| rel speed: {npc_car[2]:<6.2f}| LL car #: {len(cars_left_lane)}| RL car #: {len(cars_right_lane)}")



        #----------------------------------------- Plan next step -------------------------------

        # ______ From here on out, we are currently in a steady state _________

        # Check for impending collision:
        closest_impact_time = None

        # Utility to find min tti and min absolute gap in a list
        def analyze_lane_list(lst):
            min_tti = None
            min_gap = None
            for (rx, rs, tti) in lst:
                if min_gap is None or rx < min_gap:
                    min_gap = rx
                if tti is not None:
                    if min_tti is None or tti < min_tti:
                        min_tti = tti
            return min_gap, min_tti

        right_min_gap, right_min_tti = analyze_lane_list(cars_right_lane)
        left_min_gap, left_min_tti = analyze_lane_list(cars_left_lane)

        # Safety checks use absolute gap first (immediate)
        # When anticipating moving right:
        if anticipated_lane_right:
            # If immediate right lane car is dangerously close, try to move left (if free)
            if right_min_gap is not None and right_min_gap < self.MIN_ABS_GAP_M and not cars_left_lane:
                self.state = States.MOVING_LEFT
                # immediate return: start moving now
                desired_y = -self.LANE_MIDPOINT_OFFSET
                return desired_y, desired_speed

            # Otherwise check time-to-impact for cars ahead in our lane
            if right_min_tti is not None and (right_min_tti < FOLLOWING_DISTANCE):
                # If left lane is clear, pass
                if not cars_left_lane:
                    self.state = States.MOVING_LEFT
                else:
                    # slow down to match leader speed (use rel_speed of closest)
                    # pick the minimal tti occuring car to compute rel_speed
                    # find that car
                    min_tti = right_min_tti
                    for (rx, rs, tti) in cars_right_lane:
                        if tti == min_tti:
                            desired_speed = speed + rs
                            break
        else:
            # anticipating left lane
            if left_min_gap is not None and left_min_gap < self.MIN_ABS_GAP_M and not cars_right_lane:
                self.state = States.MOVING_RIGHT
                desired_y = self.LANE_MIDPOINT_OFFSET
                return desired_y, desired_speed

            if left_min_tti is not None and (left_min_tti < FOLLOWING_DISTANCE):
                if not cars_right_lane:
                    self.state = States.MOVING_RIGHT
                else:
                    min_tti = left_min_tti
                    for (rx, rs, tti) in cars_left_lane:
                        if tti == min_tti:
                            desired_speed = speed + rs
                            break

        # find the desired y from desired lane movement
        if (self.state == States.MOVING_LEFT):
            desired_y = -self.LANE_MIDPOINT_OFFSET
        elif (self.state == States.MOVING_RIGHT): 
            desired_y = self.LANE_MIDPOINT_OFFSET
        elif (self.state == States.STRAIGHT):
            #Keep lane
            if anticipated_lane_right:
                desired_y = self.LANE_MIDPOINT_OFFSET
            else:
                desired_y = -self.LANE_MIDPOINT_OFFSET        

        return desired_y, desired_speed



    # speed, y, phi: ego vehicle's speed, lateral position, heading
    # desired_speed: user defined speed setpoint (flexible by plus minus 3 degrees up to speed limits)
    # des_lane: desired lane specified by the user. -1 is left lane, +1 is right lane
    # other_cars: each row of this list corresponds to one of the other vehicles on the road
    #             the columns are: [relative longitudinal position, relative lateral position, relative speed]
    # grade is a road grade object. Use grade.grade_at to find the road grade at any x
    def update(self, speed, x, y, phi, desired_speed, des_lane, other_cars, grade):
        #fuel optimization MPC. must not be allowed to override safety decisions
        v_driver = desired_speed
        #for extra safety, pass MPC a safety_gap value consistent with controller settings
        v_des_opt = mpc_select_v_des(x, speed, grade, other_cars, v_driver, safety_gap_min=self.SAFETY_MIN_GAP_PRED)
        #never go faster than what the planner suggested, MPC may suggest slower speeds for fuel saving,

        v_des_opt = min(v_des_opt, desired_speed)
        self.last_v_des_opt = v_des_opt


        
        # Path planning & lane-change decision
        # NOTE: state_machine uses current 'speed' and raw other_cars (not affected by MPC)
        self.desired_y, desired_speed = self.state_machine(y, other_cars, v_des_opt, speed)
        #self.desired_y, desired_speed = self.state_machine(y, other_cars, 27.78, speed) # Kept for easy checking for instructor =) (uncomment for max speed to demo more lane changes if needed)


        self.delta_cmd= self.y_controller.update(self.desired_y, y, phi)

       

        self.Fd_cmd = self.v_controller.update(desired_speed, speed)

        #self.Fd_cmd = self.v_controller.update(desired_speed, speed) #used before fuel optimization)
        
        return self.Fd_cmd, self.delta_cmd