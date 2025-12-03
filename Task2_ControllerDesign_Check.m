%% Define Plant
s = tf('s');
P = 1 / (1300*s + 31.112);

%% Your Current PI Controller Gains
Kp = 4323.888;
Ki = 3647.3125;

% PI controller
C = Kp + Ki/s;

zero(C)

pole(C)
%% Precompensator Ki / (Kp*s + Ki)
Kpre = Ki / (Kp*s + Ki);

%% Combined Controller (if needed)
C_total = C * Kpre;

%% Open PID Tuner with your PI controller structure
% PID Tuner supports PID/PI/PIDF, so we pass 'pi'.
pidTuner(P, C);
