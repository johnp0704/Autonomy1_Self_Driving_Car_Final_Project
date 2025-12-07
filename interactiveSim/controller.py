
from PID import PID
from car import Car
import numpy as np

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

    def update(self, desired_speed, speed):
        V_ref_filtered = self.precomp.filter(desired_speed)
        
        force = self.PI.update(V_ref_filtered, speed)
        
        return force


# Task 3 Controllers
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

        self.precomp = Precompensator(Kp=Kp_outer, Ki=Ki_outer, Ts=Ts, initial_setpoint_ref=0) # Precomp



    def update(self, y_des, y, phi):

        y_filt = self.precomp.filter(y_des)

        phi_des = self.c_outer.update(y_filt, y)

        delta = self.c_inner.update(phi_des, phi)
    
        return delta



class Controller:

    LANE_MIDPOINT_OFFSET = 11.25
    LANE_THRESH = LANE_MIDPOINT_OFFSET*0.9
    LANE_DIFF_LATERAL_READING = 45 # The reading when a car is in another lane
    LAT_READING_CONVERSION_FACTOR = (2*LANE_MIDPOINT_OFFSET)/LANE_DIFF_LATERAL_READING # Conversion factor for other car rel y to meters

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
        

    def time_to_impact(self, rel_x, rel_speed, car_anticipate_right_lane, npc_in_right_lane):
        
        # Check if we anticipate being in the same lane as the other car
        if car_anticipate_right_lane != npc_in_right_lane:
            #ignore
            return None
        
        else:
            # same lane, check if other car is faster
            if rel_speed >= 0 or rel_x < 0:
                return None
            return abs(rel_x/rel_speed)
    

    def state_machine(self, y, other_cars, desired_speed, speed):
        FOLLOWING_DISTANCE = 6 #sec
        MIN_VERT_FOR_LANE_CHANGE = 11 # meters
        
        #init return, desired_speed defaults to input value
        desired_y = None
        anticipated_lane_right = None

        # -------------------------------------- Check if car needs to finish lane change --------------------------
                # Check if a previously started lane change was completed if not complte that turn first
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


        # Check if we are in the right lane and going straight, or if we are moving to it.
        if (self.state == States.MOVING_RIGHT) or ((y >= self.LANE_THRESH)
                                                   and (self.state == States.STRAIGHT)):
            anticipated_lane_right = True
        elif (self.state == States.MOVING_LEFT) or ((y <= -self.LANE_THRESH)
                                                   and (self.state == States.STRAIGHT)):
            anticipated_lane_right = False
        else:
            print("Lane determining error")





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
            rel_speed = npc_car[2]
            abs_y = y + rel_y

            # Find which lane the NPC car is in
            if abs_y > self.LANE_THRESH:
                # In right lane
                cars_right_lane.append((rel_x, rel_speed, self.time_to_impact(rel_x, rel_speed, anticipated_lane_right, True))) # Last param is antsipate car in right lane

            elif abs_y < -self.LANE_THRESH:
                # In left lane
                cars_left_lane.append((rel_x, rel_speed, self.time_to_impact(rel_x, rel_speed, anticipated_lane_right, False))) # False for car in left lane

            else:
                print("NPC CAR LANE LOCATION ERROR")
            
            print(f"Car Y: {y:<6.2f}| Rel x: {npc_car[0]:<6.2f}| Rel y: {npc_car[1]* self.LAT_READING_CONVERSION_FACTOR:<6.2f}| rel speed: {npc_car[2]:<6.2f}| LL car #: {len(cars_left_lane)}| RL car #: {len(cars_right_lane)}")






        #----------------------------------------- Plan next step -------------------------------

        # ______ From here on out, we are currently in a steady state _________

        # Check for impending collision:
        closest_impact_time = None

        # In Right lane
        if anticipated_lane_right:
            for NPC_car in cars_right_lane:
                time_to_impact = NPC_car[2]
                rel_speed = NPC_car[1]

                #TODO check why not passing when left lane seems clear
                if time_to_impact is not None and not cars_left_lane:
                    if (time_to_impact < FOLLOWING_DISTANCE):
                        self.state = States.MOVING_LEFT
                        print("moving left to pass")
                    
                else:
                    # Other lane not safe need to slow down
                    if closest_impact_time is not None and closest_impact_time > time_to_impact:
                        if (time_to_impact < FOLLOWING_DISTANCE):
                            desired_speed = speed + rel_speed
                            closest_impact_time = time_to_impact


        # In Left Lane
        else:
            
            # always try to get out of passing lane
            if not cars_right_lane:
                self.state = States.MOVING_RIGHT
                print("returning to regular lane from passing lane")



            for NPC_car in cars_left_lane:
                time_to_impact = NPC_car[2]
                rel_speed = NPC_car[1]

                # Check if we are headed to crash and if right lane is clear
                if time_to_impact is not None and not cars_right_lane:
                    if (time_to_impact < FOLLOWING_DISTANCE):
                        self.state = States.MOVING_RIGHT
                        print("moving right to pass")
                    
                else:
                    # Other lane not safe need to slow down
                    if closest_impact_time is not None and closest_impact_time > time_to_impact:
                        if (time_to_impact < FOLLOWING_DISTANCE):
                            desired_speed = speed + rel_speed
                            closest_impact_time = time_to_impact
                


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

        #TODO optimization based fuel speed control up here. It will be passed into the section below to make sure that it stays safe


        self.desired_y = des_lane * 11.25 # dist from center to lane
        self.vdes = desired_speed

        # Todo path planning
        self.desired_y, desired_speed = self.state_machine(y, other_cars, desired_speed, speed)


        self.delta_cmd= self.y_controller.update(self.desired_y, y, phi)
        self.Fd_cmd = self.v_controller.update(desired_speed, speed) 

        return self.Fd_cmd, self.delta_cmd
    
