import car


Froll = 100 #𝑁
a = 0.2 #𝑁𝑠^2/𝑚2
b = 20 #𝑁𝑠/𝑚
g = 9.8 #𝑚/𝑠^2
fd_min = -7000 #𝑁
zeta = 0.95
eta_g = 0.8
eta_d = 3.8
rw = 0.34 #𝑚
F_bar =  200 #𝑚𝑔/𝑠
Te_max = 200 #Nm
F_max = (Te_max * eta_g * eta_d) / rw * zeta #mg/s from engine
F_min = -7000 #mg/s from brakes
L = 2.7 #m
delta_max = 0.05 #rad
step_size = 300 #steps per second

v0 = 27.78

print(f"C = {(a*v0**2 + b*v0)/v0}")