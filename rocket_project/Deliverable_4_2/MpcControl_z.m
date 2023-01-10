classdef MpcControl_z < MpcControlBase
    properties
        A_bar, B_bar, C_bar % Augmented system for disturbance rejection
        L                   % Estimator gain for disturbance rejection
    end
    
    methods
        function mpc = MpcControl_z(sys, Ts, H)
            mpc = mpc@MpcControlBase(sys, Ts, H);
            
            [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
        end
        
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   d_est        - disturbance estimate
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing
            
            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.3)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar(1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system

            % constraints
            Hu = [1; -1];
            hu = [80-56.66667; -(50-56.66667)];

            % costs for the LQR controller
%             Q = diag([10, 1000]);   % sharp EPFL
%             R = diag(1);            % sharp EPFL
            Q = diag([10, 200]);    % good EPFL
            R = diag(0.0001);       % good EPFL
%             Q = diag([5, 700]);   % ~weights from 3.1
%             R = diag(0.1);        % ~weights from 3.1

            % K is the LQR controller, P is the final cost
            [K,Pf,~] = dlqr(mpc.A, mpc.B, Q, R);
            K = -K;
            Ak = mpc.A+mpc.B*K;

            % the combined constraints of state and input with controller K
            % in closed loop
            Hxu = Hu*K;
            hxu = hu;

            % the terminal set of the controller K in closed loop
            Poly_xu = polytope(Hxu, hxu);
            term_set = max_contr_invar_set(Poly_xu, Ak);
            [Hxf, hxf] = double(term_set); % terminal constraint

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];

            for k = 1:N-1
                con = [con, X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k)];   % here X is already x*-xs because we are in the linearized shifted system
                con = [con, Hu*U(:,k) <= hu];
                obj   = obj + (X(:,k)-x_ref)'*Q*(X(:,k)-x_ref) + (U(:,k)-u_ref)'*R*(U(:,k)-u_ref);
            end
            obj = obj + (X(:,N)-x_ref)'*Pf*(X(:,N)-x_ref);
            con = [con, Hxf*X(:,N) <= hxf + Hxf*x_ref];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), {X(:,1), x_ref, u_ref, d_est}, {U(:,1), X, U});
        end
        
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);
            
            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.3)
            ref = sdpvar;
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

            % constraints
            Hu = [1; -1];
            hu = [80-56.66667; -(50-56.66667)];


            Qt_pos = 1;  % Q target position
            obj = (mpc.C*xs-ref)'*Qt_pos*(mpc.C*xs-ref);
            con = [xs == mpc.A*xs + mpc.B*us, Hu*us <= hu];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
        end
        
        
        % Compute augmented system and estimator gain for input disturbance rejection
        function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
            
            %%% Design the matrices A_bar, B_bar, L, and C_bar
            %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
            %%% converges to the correct state and constant input disturbance
            %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            
            A_bar = [];
            B_bar = [];
            C_bar = [];
            L = [];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        
    end
end
