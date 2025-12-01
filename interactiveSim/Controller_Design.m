% Task 3

v0 = 27.78;
L = 2.7;

P_psi_from_delta = tf(v0/L,[1, 0]);

P_y_from_psi = tf(v0,[1, 0]);


% first plant (inner Loop)
pidTuner(P_psi_from_delta,"P")

% Picked from output
% C_inner.Kp = 1;
%%
s = tf("s");

% Update model with pidtuner value
% oltf_inner = C_inner.Kp * P_psi_from_delta;


%overide with kp_inner = 1
oltf_inner = (2) * P_psi_from_delta;

cltf_inner = feedback(oltf_inner,1);

P_outer = cltf_inner * P_y_from_psi;


pidTuner(P_outer)

%% RL

% c_rl = tf([1, ki_kp], [1,0]);
% c_rl = (s+7.5)*(s+15) * (s+0.5)/...
%           (s);

c_rl = (s+10.2889)*(s+7.5) ...* (s+0.5)...
           /(s);

rlocus(c_rl*P_outer)


%%
%From Rlplot
k = 0.1;

Outer_CLTF = feedback(k*c_rl*P_outer, 1);

step(Outer_CLTF)