%% Define Plant
s = tf('s');
P = 1 / (1300*s + 31.112);

%% Your Current PI Controller Gains
Kp = 4323.888;
Ki = 3647.3125;

% PI controller
C = Kp + Ki/s;

% Display Controller Zero and Pole
zero(C)
pole(C)

%% Precompensator Ki / (Kp*s + Ki)
Kpre = Ki / (Kp*s + Ki);

%% Combined Controller (if desired)
C_total = C * Kpre;

%% --------- Plot Gain and Phase Margins (OPEN LOOP ONLY) ---------
L = C * P;  % open-loop transfer function

figure;
margin(L);          % Bode with GM/PM markers
grid on;
title('Open-Loop Gain and Phase Margins');
