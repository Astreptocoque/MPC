classdef MpcControl_x < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % construct the polytopes
            Hx = [0 1 0 0; 0 -1 0 0];
            Hu = [1; -1];
            hx = [deg2rad(7); deg2rad(7)];
            hu = [deg2rad(15); deg2rad(15)];

            % costs for the LQR controller
            Q = diag([5, 15, 1, 5]);
            R = diag(5);

            % K is the LQR controller, P is the final cost
            [K,Pf,~] = dlqr(mpc.A, mpc.B, Q, R);
            K = -K;
            Ak = mpc.A+mpc.B*K;

            % the combined constraints of state and input with controller K
            % in closed loop
            Hxu = [Hx; Hu*K];
            hxu = [hx; hu];

            % the terminal set of the controller K in closed loop
            Poly_xu = polytope(Hxu, hxu);
            term_set = max_contr_invar_set(Poly_xu,Ak);
            [Hxf, hxf] = double(term_set); % terminal constraint

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];

            for k = 1:N-1
                con = [con, X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k)];
                con = [con, Hu*U(:,k) <= hu, Hx*X(:,k) <= hx];
                obj   = obj + (X(:,k)-x_ref)'*Q*(X(:,k)-x_ref) + (U(:,k)-u_ref)'*R*(U(:,k)-u_ref);
            end
            obj = obj + (X(:,N)-x_ref)'*Pf*(X(:,N)-x_ref);
            con = [con, Hxf*X(:,N) <= hxf+Hxf*x_ref];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

            Hx = [0 1 0 0; 0 -1 0 0];
            Hu = [1; -1];
            hx = [deg2rad(7); deg2rad(7)];
            hu = [deg2rad(15); deg2rad(15)];
            
            Q = eye(nx);
            obj = (xs - ref.*[0;0;0;1])'*Q*(xs - ref.*[0;0;0;1]);
            con = [xs == mpc.A*xs + mpc.B*us, Hx*xs <= hx, Hu*us <= hu];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
