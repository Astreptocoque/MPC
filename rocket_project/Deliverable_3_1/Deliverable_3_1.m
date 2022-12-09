addpath(fullfile('..', 'src'));
%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
% Design MPC controller
H = 10; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input
u_x = mpc_x.get_u(x)

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T opt, X opt, U opt] = mpc_x.get u(x);
U opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);