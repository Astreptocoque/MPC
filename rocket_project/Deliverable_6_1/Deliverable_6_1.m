%% Setup path to have no conflict
addpath(fullfile('src'));
addpath(fullfile('Deliverable_4_2'),"-begin")
warning('off','MATLAB:rmpath:DirNotFound')
rmpath(fullfile('Deliverable_3_1'))
rmpath(fullfile('Deliverable_3_2'))
rmpath(fullfile('Deliverable_4_1'))
rmpath(fullfile('Deliverable_4_2'))
rmpath(fullfile('Deliverable_5_1'))

clear; close all;

%% This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x = zeros(12,1); x0 = zeros(12,1);
ref = [10; 10; 10; deg2rad(45)];
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);

% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_EPFL(t_);
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
% ref = @(t_, x_) ref_EPFL(t_, roll_max);

Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);