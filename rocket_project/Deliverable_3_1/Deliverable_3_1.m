%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
x0 = [0; pi/4]; % initial state

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% Simulate the roll system
% Design MPC controller
H = 4; % Horizon length in seconds
% for H<3 there is overshoot
sim_duration = 10;
num_steps = sim_duration/Ts;
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Get control input
u_roll = mpc_roll.get_u(x0);

% simulate open-loop trajectory (apply controller once and use the whole
% input sequence)
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual

% simulate in closed loop (apply the controller after every step and use
% only the first control input)
U_closed_loop = zeros(1,sim_duration/Ts+1);
X_closed_loop = zeros(2,sim_duration/Ts+1);
X_closed_loop(:,1) = x0;
for i=1:num_steps
    U_closed_loop(:,i) = mpc_roll.get_u(X_closed_loop(:,i));
    X_closed_loop(:,i+1) = mpc_roll.A*X_closed_loop(:,i) + mpc_roll.B*U_closed_loop(:,i);
end
ph = rocket.plotvis_sub(0:Ts:sim_duration, X_closed_loop, U_closed_loop, sys_roll, xs, us); % Plot as usual

% simulate the system closed loop (solution)
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0, sim_duration, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);

%% Simulate the x system






