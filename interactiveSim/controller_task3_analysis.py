import numpy as np
import matplotlib.pyplot as plt
import control as ct


v0 = 27.78
L = 2.7

#-------------outer-------------
#gains
Ki = 0.4039776818
Kp = 0.2411807055

print(f'\nUsing Controller Gains:')
print(f'Kp = {Kp}')
print(f'Ki = {Ki}')

s = ct.TransferFunction.s

#plant
P = v0 / s

#controller
C = (Kp * s + Ki) / s

TF = C * P

#bode plot
plt.figure()
ct.bode_plot(TF, dB=True, display_margins=True)
plt.suptitle(f'Bode Plot of Transfer Function')

#closed-loop TF
T_feedback = ct.feedback(TF, 1)

#precompensator
F = (Ki/Kp) / (s + Ki/Kp)

# Final transfer function
T_final = F * T_feedback

#step response
t = np.linspace(0, 10, 2000)
t_resp, y = ct.step_response(T_final, T=t)

y_final = y[-1]
y_max = np.max(y)

# Overshoot
overshoot = max(0, (y_max - y_final) / y_final * 100)

# Rise time 10–90%
t_10 = np.where(y >= 0.1 * y_final)[0][0]
t_90 = np.where(y >= 0.9 * y_final)[0][0]
rise_time = t_resp[t_90] - t_resp[t_10]

# Settling time (2%)
upper = 1.02 * y_final
lower = 0.98 * y_final
outside = (y > upper) | (y < lower)
if np.any(outside):
    last_idx = np.where(outside)[0][-1]
    settling_time = t_resp[last_idx]
else:
    settling_time = 0

# --- Print Metrics ---
print(f'\nPerformance Metrics:')
print(f'Final Value: {y_final:.4f}')
print(f'Overshoot: {overshoot:.2f}%')
print(f'Rise Time: {rise_time:.4f} s')
print(f'Settling Time: {settling_time:.4f} s')

# --- Plot Step Response ---
plt.figure()
plt.plot(t_resp, y)
plt.axhline(1.0, color='r', linestyle='--', label='Reference')
plt.title('Step Response with Prefiltered Closed Loop')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.grid(True)
plt.legend()

plt.show()

#------------Inner---------------
#gains
Kp_inner = 1

print(f'\nUsing Controller Gains:')
print(f'Kp = {Kp_inner}')

s = ct.TransferFunction.s

#plant
P_inner = v0/L * 1/s

#controller
C_inner = Kp

TF_inner = C_inner * P_inner

#bode plot
plt.figure()
ct.bode_plot(TF_inner, dB=True, display_margins=True)
plt.suptitle(f'Bode Plot of Inner Transfer Function')

#closed-loop TF
T_feedback_inner = ct.feedback(TF_inner, 1)


#step response
t_inner = np.linspace(0, 10, 2000)
t_resp_inner, y_inner = ct.step_response(T_feedback_inner, T=t_inner)

y_final_inner = y_inner[-1]
y_max_inner = np.max(y_inner)

# Overshoot
overshoot_inner = max(0, (y_max_inner - y_final_inner) / y_final_inner * 100)

# Rise time 10–90%
t_10 = np.where(y_inner >= 0.1 * y_final_inner)[0][0]
t_90 = np.where(y_inner >= 0.9 * y_final_inner)[0][0]
rise_time_inner = t_resp_inner[t_90] - t_resp_inner[t_10]

# Settling time (2%)
upper_inner = 1.02 * y_final_inner
lower_inner = 0.98 * y_final_inner
outside_inner = (y_inner > upper_inner) | (y_inner < lower_inner)
if np.any(outside_inner):
    last_idx_inner = np.where(outside_inner)[0][-1]
    settling_time_inner = t_resp_inner[last_idx_inner]
else:
    settling_time_inner = 0

# --- Print Metrics ---
print(f'\nPerformance Metrics:')
print(f'Final Value: {y_final_inner:.4f}')
print(f'Overshoot: {overshoot_inner:.2f}%')
print(f'Rise Time: {rise_time_inner:.4f} s')
print(f'Settling Time: {settling_time_inner:.4f} s')

# --- Plot Step Response ---
plt.figure()
plt.plot(t_resp_inner, y_inner)
plt.axhline(1.0, color='r', linestyle='--', label='Reference')
plt.title('Step Response with Prefiltered Closed Loop')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.grid(True)
plt.legend()

plt.show()