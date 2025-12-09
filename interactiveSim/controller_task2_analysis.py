import numpy as np
import matplotlib.pyplot as plt
import control as ct

m = 1300.0
a = 0.2
b = 20.0
v0 = 27.78
c = 2 * a * v0 + b
print(f'Linearized damping coefficient = {c:.4f} Ns/m')

zeta = 1.0 #critically damped
omega_n = 1.675 #chosen

#gain calculations (coefficient matching)
Ki = m * (omega_n**2)
Kp = 2 * zeta * omega_n * m - c

Ki = Ki/2
Kp = Kp/2

print(f'\nDesigned Controller Gains:')
print(f'Kp: {Kp:.4f}')
print(f'Ki: {Ki:.4f}')

s = ct.TransferFunction.s #setup transfer functions
P = 1 / (m * s + c) #plant
C = (Kp * s + Ki) / s #controller (PI)
TF = C * P #transfer function

#bode plot
plt.figure()
ct.bode_plot(TF, dB=True, display_margins=True)
plt.suptitle(f'Bode Plot of Transfer Function')

F = Ki / (Kp * s + Ki) #pre-comp

T_feedback = ct.feedback(TF, 1)
T_final = F * T_feedback

#step Response
t, y = ct.step_response(T_final, T=np.linspace(0, 10, 1000))
y_final = y[-1]
y_max = np.max(y)

#overshoot calculation
overshoot = (y_max - y_final) / y_final * 100 #percentage
if overshoot < 0: overshoot = 0 #if no overshoot

#rise time calculation
t_10_id = np.where(y >= 0.1 * y_final)[0][0] #index where response crosses 10%
t_90_id = np.where( y >= 0.9 * y_final)[0][0] #index where response crosses 90%
rt = t[t_90_id] - t[t_10_id]

#settling time calculation
upper = 1.02 * y_final #2% above
lower = 0.98 * y_final #2% below
outside = (y > upper) | (y < lower)
if np.any(outside):
    last_violation = np.where(outside)[0][-1]
    settling_time = t[last_violation + 1] if last_violation < len(t)-1 else t[-1]
else:
    settling_time = 0.0

print(f'\nPerformance Metrics:')
print(f'Final Value: {y_final:.4f} | (Req: 1 for step input)')
print(f'Overshoot: {overshoot:.2f}% | (Req: 0%)')
print(f'Rise Time: {rt:.4f} s | (Req: 1-3 s)')
print(f'Settling Time: {settling_time:.4f} s | (Req: <10 s)')


plt.figure()
plt.plot(t, y)
plt.axhline(1.0, color='r', linestyle='--', label='Target')
plt.title(f'Closed Loop Step Response (Half-Gains)')
plt.xlabel('Time (s)')
plt.ylabel('Speed (normalized)')
plt.grid(True)
plt.legend()

plt.show()