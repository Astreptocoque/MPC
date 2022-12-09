%%
Ts = 1/20;
rocket = Rocket(Ts);
Tf = 3.0; % Simulation end time
x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
u = [deg2rad([0.1 0]), 63, 0 ]'; % (d1 d2 Pavg Pdiff) Constant input
[T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);

%%
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim poin

%%
H = 8; % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_x, Ts, H);

% Get control input
u_roll = mpc_roll.get_u(x);