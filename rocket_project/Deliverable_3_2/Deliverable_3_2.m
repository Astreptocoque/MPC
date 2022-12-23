%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();   % stable point
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);


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
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us, pos_ref);

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

% simulate the system closed loop (solution)
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0, sim_duration, @mpc_z.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, pos_ref);
