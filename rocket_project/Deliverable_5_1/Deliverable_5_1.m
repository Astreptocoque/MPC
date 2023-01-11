%% Setup path to have no conflict
addpath(fullfile('src'));
addpath(fullfile('Deliverable_5_1'),"-begin")
warning('off','MATLAB:rmpath:DirNotFound')
rmpath(fullfile('Deliverable_3_1'))
rmpath(fullfile('Deliverable_3_2'))
rmpath(fullfile('Deliverable_4_1'))
rmpath(fullfile('Deliverable_4_2'))
rmpath(fullfile('Deliverable_6_1'))

clear; close all;

%% This file should produce all the plots for the deliverable
Ts = 1/20;
H = 4;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
% x0 = zeros(12,1);
% ref4 = [2 2 2 deg2rad(40)]';
% [u, T_opt, X_opt, U_opt] = mpc.get_u(x0, ref4);
% U_opt(:,end+1) = nan;
% ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4); % Plot as usual

rocket.mass = 1.794; % Manipulate mass for simulation

% Setup reference function
ref = @(t_, x_) ref_EPFL(t_);

% Simulate
Tf = 30;
x0 = zeros(12,1);
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with mass compensation'; % Set a figure title
exportgraphics(ph.fig, "Deliverable_5_1/Figures/5.1_with_mass_compensation.png")
