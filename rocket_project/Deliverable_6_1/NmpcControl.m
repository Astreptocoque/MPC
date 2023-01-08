classdef NmpcControl < handle
    
    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
    methods
        function obj = NmpcControl(rocket, tf)
           
            import casadi.*
            
            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
            
            % Default state and input constraints
            ubx = inf(nx, 1);
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            ubx(5,1) = deg2rad(80); lbx(5,1) = deg2rad(-80); % beta
            ubu(1,1) = deg2rad(15); lbu(1,1) = deg2rad(-15); % d1
            ubu(2,1) = deg2rad(15); lbu(2,1) = deg2rad(-15); % d2
            ubu(3,1) = 80; lbu(3,1) = 50; % Pavg
            ubu(4,1) = 20; lbu(4,1) = -20; % Pdiff

            % Cost
%             Hu = [1; -1]; hu = [80-56.66667; -(50-56.66667)];
            Q = 10.*eye(nx);
%             Q(1,1) = Q(1,1)*1; % speed
%             Q(2,2) = Q(2,2)*100; % position
            R = eye(nu);
            xyzb = [10 11 12 5];
            dq = diag(Q);
            Qq = diag(dq(xyzb));

            % Terminal cost
%             sys = rocket.linearize();
%             [K,Pf,~] = dlqr(sys.A, sys.B, Q, R); K = -K;
%             Ak = sys.A + sys.B*K; % 12x12
%             Hxu = [eye(nu); -eye(nu)]; hxu = [ubu; lbu]; % 8x4, 4x1
%             Poly_xu = polytope(Hxu, hxu);
%             term_set = max_contr_invar_set(Poly_xu, Ak);
%             [Hxf, hxf] = double(term_set); % terminal constraint

            % Discretization
            h = rocket.Ts;

            eq_constr = []; % Equality constraints (Casadi SX), each entry == 0
            ineq_constr = []; % Inequality constraints (Casadi SX), each entry <= 0
            cost = 0;
            for k = 1:N-1
                k1 = rocket.f(X_sym(:,k), U_sym(:,k));
                k2 = rocket.f(X_sym(:,k) + h/2*k1, U_sym(:,k));
                k3 = rocket.f(X_sym(:,k) + h/2*k2, U_sym(:,k));
                k4 = rocket.f(X_sym(:,k) + h*k3, U_sym(:,k));
                eq_constr = [eq_constr; X_sym(:,k) + h/6*(k1 + 2*k2 + 2*k3 + k4) - X_sym(:,k+1)];
%                 ineq_constr = [ineq_constr; X_sym(:,k+1)-ubx; lbx-X_sym(:,k+1); U_sym(:,k)-ubu; lbu-U_sym(:,k)];
                cost = cost + (X_sym(xyzb,k)-ref_sym)'*Qq*(X_sym(xyzb,k)-ref_sym);
            end
%             ineq_constr = [ineq_constr; Hxf*X_sym(:,N)-hxf];
            cost = cost + (X_sym(xyzb,N)-ref_sym)'*Qq*(X_sym(xyzb,N)-ref_sym);

            % For box constraints on state and input, overwrite entries of
            % lbx, ubx, lbu, ubu defined above

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----
            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0, ref)
            
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
            
            % Additional evaluation
            % Complete trajectory
            % % X_opt = full(reshape(nlp_x(idx_X(1):idx_X(2)), obj.nx, obj.N));
            % % U_opt = full(reshape(nlp_x(idx_U(1):idx_U(2)), obj.nu, obj.N - 1));
            % %
            % % cost_opt = full(sol.f);
            % % constr_opt = full(sol.g);
            % %
            % % stats = obj.solver.stats;
        end
        
        function solve(obj, x0, ref)
            
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end

