
from PID import PID

class Controller:

    # Ts: sample time of the controller;
    # initial_conditions: two element vector representing the equilibrium 
    # i.e., [equilibrium of driving force Fd, equilibrium of vehicle speed]
    def __init__(self, Ts, initial_conditions):
        # parameters go here

        # states/controller initialization go here


    # speed, y, phi: ego vehicle's speed, lateral position, heading
    # desired_speed: user defined speed setpoint (flexible by plus minus 3 degrees up to speed limits)
    # des_lane: desired lane specified by the user. -1 is left lane, +1 is right lane
    # other_cars: each row of this list corresponds to one of the other vehicles on the road
    #             the columns are: [relative longitudinal position, relative lateral position, relative speed]
    # grade is a road grade object. Use grade.grade_at to find the road grade at any x
    def update(self, speed, y, phi, desired_speed, des_lane, other_cars, grade):

        ....

        self.desired_y = 
        self.vdes = 

        self.delta_cmd = 
        self.Fd_cmd = 

        
        return self.Fd_cmd, self.delta_cmd
