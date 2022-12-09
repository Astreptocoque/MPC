Ts = 1/20;
rocket = Rocket(Ts);
u = [deg2rad([0, 0]), 50, 0];  %[d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)
x = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]'; %[w, phi, v, p]'; % (Assign appropriately)
x_dot = rocket.f(x, u)