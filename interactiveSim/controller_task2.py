from PID import PID

class Precompensator:
    def __init__(self, Kp, Ki, Ts, initial_v_ref):
        self.KP = Kp
        self.Ki = Ki
        self.Ts = Ts

        denom = Kp + Ki * Ts
        self.alpha = (Ki * Ts) / denom
        self.beta = Kp / denom
        self.Vf_prev = initial_v_ref

    def filter(self, V_ref_current):
        Vf_current = self.alpha * V_ref_current + self.beta * self.Vf_prev
        self.Vf_prev = Vf_current
        return Vf_current
    
class controller_t2:

    def __init__(self, Kp=4323.888, Ki=3647.3125, Ts=1/60, umax=10000, umin=-10000, Kaw=60):
        #Kaw = 1.0/Ts #standard value

        v0 = 27.78
        a = 0.2
        b = 20
        F_roll = 100
        F_drag = F_roll + a * v0**2 + b * v0
        #insantiate controller
        self.PI = PID(Kp=Kp, Ki=Ki, Ts=Ts, umax=umax, umin=umin, Kaw=Kaw, initialState=F_drag)
        self.precomp = Precompensator(Kp=Kp, Ki=Ki, Ts=Ts, initial_v_ref=v0)

    def update(self, speed, desired_speed):
        V_ref_filtered = self.precomp.filter(desired_speed)
        force = self.PI.update(V_ref_filtered, speed)
        return force