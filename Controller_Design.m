% Task 3

v0 = 27.78;
L = 2.7;

P_psi_from_delta = tf(v0/L,[1, 0]);

P_y_from_psi = tf(v0,[1, 0]);


% first plant (inner Loop)
% pidTuner(P_psi_from_delta,"P")

% Picked from output
C_inner.Kp = 2
%%
s = tf("s");

% Update model with pidtuner value
oltf_inner = C_inner.Kp * P_psi_from_delta;


%overide with kp_inner = 1
% oltf_inner = 1 * P_psi_from_delta;

cltf_inner = feedback(oltf_inner,1);

P_outer = cltf_inner * P_y_from_psi;


% pidTuner(P_outer, "PI")
% pidTuner(P_y_from_psi, "PI")


%% hand Calc 

P_loc = 3;
kp = 2*P_loc/v0

ki = v0*kp^2/4

Controller_outer = (kp + ki/s)

cltf_outer = feedback(Controller_outer * P_outer, 1);

precomp_loc = zero(Controller_outer);

precomp = -precomp_loc/(s-precomp_loc)


step(precomp*cltf_outer)

%% Precomp
% Controller_outer = (C_outer_2.Kp + C_outer_2.Ki/s);
% 
% cltf_outer = feedback(Controller_outer * P_outer, 1);
% 
% precomp_loc = zero(Controller_outer)
% 
% precomp = -precomp_loc/(s-precomp_loc)
% 
% 
% step(precomp*cltf_outer)


