%addpath(fullfile('..', 'src'));
%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
x0 = [0; pi/2];
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% Simulate the roll system
% Design MPC controller
H = 10; % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Get control input
u_roll = mpc_roll.get_u(x0);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual

% simulate the system (dont trust this for now)
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);


