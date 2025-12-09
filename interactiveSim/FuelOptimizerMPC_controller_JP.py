import math
import numpy as np

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
    '''Steady-state fuel burn rate in mg/s for speed v and grade beta'''
    Fd = F_d_ss(v, beta)
    # engine torque and speed
    Te = (1.0/zeta) * (rw / (eta_g * eta_d)) * Fd
    Ne = (60.0/(2.0*math.pi)) * (eta_g*eta_d / rw) * v
    bsfc = bsfc_from_Ne_Te(Ne, Te)
    fr = (1.0/zeta) * bsfc * Fd * v
    return max(fr, F_bar)

def predict_other_positions_global(x0, other_cars, k, Ts, v0):
    '''
    Predict global positions of other cars after k steps.
    other_cars rows: [rel_x, rel_y, rel_speed], where rel_x is distance ahead of ego (m),
    rel_speed = other_speed - ego_speed.
    Global other position at t0 = x0 + rel_x
    other_speed = v0 + rel_speed
    '''
    preds = []
    for oc in other_cars:
        other_speed = v0 + oc[2]
        other_x_k = (x0 + oc[0]) + other_speed * (k * Ts)
        preds.append(other_x_k)
    return preds

def mpc_cost_for_vdes(v_des_candidate, x0, v0, grade_obj, other_cars, v_driver, Ts=Ts_mpc, N=N, tau=tau_v, safety_gap_min=16.0):
    '''Simulate coarse dynamics forward for constant v_des_candidate and return total cost.
       This function can be used for a grid search over v_des candidates. '''
    x = x0
    v = v0
    total_cost = 0.0

    # Quick check: if any current other car is already closer than safety gap, disallow candidates that reduce gap
    for oc in other_cars:
        if oc[0] < safety_gap_min:
            # If candidate speed would decrease the ego's ability to maintain gap, give huge cost
            # We'll still allow candidates that increase the gap (i.e., larger v), but penalize those that make it worse
            # (This is a conservative check.)
            # We don't return immediately here; we'll later reject candidates that reduce gap in first step.

            # nothing here — we'll check in the k loop
            pass

    for k in range(N):
        # road grade at x
        beta_deg = grade_obj.grade_at(x)
        beta = math.atan(beta_deg/100.0)

        # fuel burn at current v
        Jss = fuel_rate_ss(v, beta)
        total_cost += Jss * Ts

        # Dynamics: first-order to v_des_candidate
        v = v + (Ts/tau) * (v_des_candidate - v)
        x = x + Ts * v  # predicted ego position at next step

        # Predict other vehicles' positions at this future time (global)
        for oc in other_cars:
            other_speed = v0 + oc[2]
            other_x_k = (x0 + oc[0]) + other_speed * ((k+1)*Ts)  # at end of step k
            ego_x_k = x  # predicted ego position at end of step k
            gap = other_x_k - ego_x_k

            # If other vehicle is behind us (negative gap) skip
            if gap <= 0:
                continue

            # Safety: if gap ever drops below safety threshold, reject candidate
            if gap < safety_gap_min:
                return 1e9 #huge penalty — candidate not safe

        # penalty to keep near driver desired speed
    total_cost += lambda_tracking * (v_des_candidate - v_driver)**2
    return total_cost

def mpc_select_v_des(x0, v0, grade_obj, other_cars, v_driver, search_resolution=0.1, safety_gap_min=16.0):
    # allowable band per spec: clamp (v_driver +/- 3) into [v_min, v_max]
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
