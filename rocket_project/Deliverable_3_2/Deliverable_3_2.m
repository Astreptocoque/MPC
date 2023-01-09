addpath(fullfile('src'));
addpath(fullfile('Deliverable_3_2'),"-begin")
warning('off','MATLAB:rmpath:DirNotFound')
rmpath(fullfile('Deliverable_3_1'))
rmpath(fullfile('Deliverable_4_1'))
rmpath(fullfile('Deliverable_4_2'))
rmpath(fullfile('Deliverable_5_1'))
rmpath(fullfile('Deliverable_6_1'))

clear; close all;

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();   % stable point
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);


%% Simulate the roll system
% Design MPC controller
H = 4; % Horizon length in seconds
% for H<3 there is overshoot
sim_duration = 8;
num_steps = sim_duration/Ts;
x0 = [0; 0]; % initial state
x_ref = deg2rad(35);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Get control input
u_roll = mpc_roll.get_u(x0, x_ref);

% simulate open-loop trajectory (apply controller once and use the whole
% input sequence)
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0, x_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_roll_subsystem_openloop.png")

% simulate in closed loop (apply the controller after every step and use
% only the first control input)
U_closed_loop = zeros(1,sim_duration/Ts+1);
X_closed_loop = zeros(2,sim_duration/Ts+1);
X_closed_loop(:,1) = x0;
for i=1:num_steps
    U_closed_loop(:,i) = mpc_roll.get_u(X_closed_loop(:,i), x_ref);
    X_closed_loop(:,i+1) = mpc_roll.A*X_closed_loop(:,i) + mpc_roll.B*U_closed_loop(:,i);
end
ph = rocket.plotvis_sub(0:Ts:sim_duration, X_closed_loop, U_closed_loop, sys_roll, xs, us); % Plot as usual
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_roll_subsystem_closedloop.png")

% simulate the system closed loop (solution)
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0, sim_duration, @mpc_roll.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);


%% Simulate the x system
% Design MPC controller
H = 4;
sim_duration = 8;
num_steps = sim_duration/Ts;

x0 = [0; 0; 0; 0]; % initial state
x_ref = -4;

mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input
u_x = mpc_x.get_u(x0, x_ref);

% simulate open-loop trajectory (apply controller once and use the whole
% input sequence)
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, x_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us, x_ref);
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_x_subsystem_openloop.png")

% simulate in closed loop (apply the controller after every step and use
% only the first control input)
U_closed_loop = zeros(1,sim_duration/Ts+1);
X_closed_loop = zeros(4,sim_duration/Ts+1);
X_closed_loop(:,1) = x0;

for i=1:num_steps
    U_closed_loop(:,i) = mpc_x.get_u(X_closed_loop(:,i), x_ref);
    X_closed_loop(:,i+1) = mpc_x.A*X_closed_loop(:,i) + mpc_x.B*U_closed_loop(:,i);
end
ph = rocket.plotvis_sub(0:Ts:sim_duration, X_closed_loop, U_closed_loop, sys_x, xs, us, x_ref);
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_x_subsystem_closedloop.png")

% simulate the system closed loop (solution)
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, sim_duration, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);


%% Simulate the y system
% Design MPC controller
H = 4;
sim_duration = 8;
num_steps = sim_duration/Ts;

x0 = [0; 0; 0; 0]; % initial state
x_ref = -4;

mpc_y = MpcControl_y(sys_y, Ts, H);
% Get control input
u_y = mpc_y.get_u(x0, x_ref);

% simulate open-loop trajectory (apply controller once and use the whole
% input sequence)
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(x0, x_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_y_subsystem_openloop.png")

% simulate in closed loop (apply the controller after every step and use
% only the first control input)
U_closed_loop = zeros(1,sim_duration/Ts+1);
X_closed_loop = zeros(4,sim_duration/Ts+1);
X_closed_loop(:,1) = x0;

for i=1:num_steps
    U_closed_loop(:,i) = mpc_y.get_u(X_closed_loop(:,i), x_ref);
    X_closed_loop(:,i+1) = mpc_y.A*X_closed_loop(:,i) + mpc_y.B*U_closed_loop(:,i);
end
ph = rocket.plotvis_sub(0:Ts:sim_duration, X_closed_loop, U_closed_loop, sys_y, xs, us);
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_y_subsystem_closedloop.png")

% simulate the system closed loop (solution)
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0, sim_duration, @mpc_y.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);


%% Simulate the z system
% Design MPC controller
H = 4;
sim_duration = 10;
num_steps = sim_duration/Ts;

x0 = [0; 0]; % initial state
pos_ref = -4; % reference position (final position)

mpc_z = MpcControl_z(sys_z, Ts, H);
% Get control input
u_z = mpc_z.get_u(x0);

% simulate open-loop trajectory (apply controller once and use the whole
% input sequence)
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(x0, pos_ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt+us(3), sys_z, xs, us, pos_ref);
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_z_subsystem_openloop.png")

% simulate in closed loop (apply the controller after every step and use
% only the first control input)
U_closed_loop = zeros(1,sim_duration/Ts+1);
X_closed_loop = zeros(2,sim_duration/Ts+1);
X_closed_loop(:,1) = x0;

for i=1:num_steps
    U_closed_loop(:,i) = mpc_z.get_u(X_closed_loop(:,i), pos_ref);
    X_closed_loop(:,i+1) = mpc_z.A*X_closed_loop(:,i) + mpc_z.B*(U_closed_loop(:,i));
end
ph = rocket.plotvis_sub(0:Ts:sim_duration, X_closed_loop, U_closed_loop+us(3), sys_z, xs, us, pos_ref);
exportgraphics(ph.fig, "Deliverable_3_2/Figures/3.2_z_subsystem_closedloop.png")

% simulate the system closed loop (solution)
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0, sim_duration, @mpc_z.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, pos_ref);

Ts = 1/20; % Sample time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();   % stable point
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
