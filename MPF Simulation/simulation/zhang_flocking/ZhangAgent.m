%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

% This class is a implementation of the MPC flocking algorithm in
% H. -T. Zhang, Z. Cheng, G. Chen and C. Li, 
% "Model predictive flocking control for second-order multi-agent systems with input constraints," 
% in IEEE Transactions on Circuits and Systems I: Regular Papers, vol. 62, no. 6, pp. 1599-1606, June 2015, 
% doi: 10.1109/TCSI.2015.2418871.

classdef ZhangAgent < DoubleIntegratorAgent
    
    % Define constants of the flocking protocol
    properties
        d;          % equilibrium distance(or desired distance) to neighbours
        lambda;     % weight on control action
        Hp;         % prediction horizon
        m;          % state dimensions
        u_max;      % maximum input
    end
    
    % data to be transmitted in addition to position, velocity, and u
    properties
        num_N;  % number of neighbors
        neighbors;
        U_opt;
        cost;
    end
    
    % member variables
    properties(GetAccess = private, SetAccess = private)
        epsilon;    % sampling time
        p_x;        % single agent state transition matrix
        p_u;        % single agent input matrix
        numberAgents;
    end
    
    methods(Static)
        function e_nm = e_nm(n,m)
            e_nm = zeros(1,n);
            e_nm(m) = 1;
        end        
    end
    
    methods
        %PredictiveFlockingAgent Initialize agent
        % Inputs:
        %   id          : agent id
        %   dT          : sampling time
        %   initialPos  : initial position
        %   initialVel  : initial velocity
        function obj = ZhangAgent(id, param, initialPos, initialVel, cfg)
            % call parent constructor
            obj@DoubleIntegratorAgent(id, param.dT, initialPos, initialVel);
            
            % load parameters
            obj.d = cfg.d;
            obj.lambda = cfg.lambda;
            obj.Hp = cfg.Hp;
            obj.m = cfg.m;
            obj.u_max = cfg.u_max;
            
            % initialize member variables
            obj.epsilon = param.dT;
            obj.num_N = 0;
            obj.numberAgents = param.agentCount;
            obj.U_opt = zeros(obj.Hp * obj.m,1);
            obj.cost = 0;
            
            % set up p_x and p_u for prediction horizon
            A = kron([1, obj.epsilon; 0, 1], eye(obj.m));
            B = kron([0; obj.epsilon], eye(obj.m));
            % p_x dimensions: 2*m*Hp x 2*m
            p_x = [];
            for h = 1:obj.Hp
                p_x = [p_x; A^h];
            end
            obj.p_x = p_x;
            % p_u dimensions: 2*m*Hp x m*Hp
            p_u = zeros(2 * obj.m * obj.Hp, obj.m * obj.Hp);
            for h = 1:obj.Hp
                % construct from kroneckered identity matrices that are
                % getting smaller in size
                row_index = (h - 1) * 2 * obj.m + 1;
                column_index = (obj.Hp - (h - 1)) * obj.m;
                p_u(row_index:end, 1:column_index) = p_u(row_index:end, 1:column_index)+ kron(eye(obj.Hp - (h - 1)), A^(h - 1) * B);
            end
            obj.p_u = p_u;
        end
        
        function P_x = calculatePx(obj, num_N)
            % P_x dimensions: 2*m*Hp(N+1) x 2*m*Hp(N+1)
            P_x = kron(eye(num_N+1), obj.p_x);
        end
        
        function P_u = calculatePu(obj, num_N)
            % P_u dimensions: 2*m*Hp(N+1) x m*Hp
            P_u = kron([1; zeros(num_N,1)], obj.p_u);
        end
        
        % L: desired pairwise distance matrix
        function L = calculateL(obj, xi, num_N)
            P_x = obj.calculatePx(num_N); % 2*m*Hp(N+1) x 2*m*Hp(N+1)
            P_u = obj.calculatePu(num_N); % 2*m*Hp(N+1) x m*Hp
            % calculate and reshape Xi so q_ij is easier to compute (could be removed for computation speed)
            U_prior = [obj.U_opt(obj.m+1:end); zeros(obj.m,1)];
            Xi = P_x*xi + P_u*U_prior;
            L = [];
            % iterate over prediction horizon
            for h = 1:obj.Hp
                s_i = kron(obj.e_nm(obj.num_N+1,1), eye(2*obj.m*obj.Hp));
                s_hp = kron(obj.e_nm(obj.Hp,h), eye(2*obj.m));
                x_i = s_hp*s_i*Xi;
                l_i = [];
                % iterate over neighbors
                for j = 1:num_N              
                    s_j = kron(obj.e_nm(obj.num_N+1,j+1), eye(2*obj.m*obj.Hp));                 
                    x_j = s_hp*s_j*Xi;
                    
                    % calculate pairwise distance
                    q_ij = kron([1 0],eye(obj.m))*(x_j-x_i);
                    
                    % calculate desired distance
                    l_i = [l_i; obj.d * q_ij / norm(q_ij)];
                end
                L = [L;l_i];
            end
        end
        
        function C = calculateC(obj, num_N)
            C = [];
            eye_Hp = eye(obj.Hp);
            for h = 1:obj.Hp
                c_i = [];
                for j  = 1:num_N
                    % set value at position of agent i to -1
                    c_ij = [-1, zeros(1 ,num_N)];
                    % set value at position of neighbor j to 1
                    c_ij(j+1) = 1;
                    % extend c to prediction horizon
                    %                     c_ij_Hp = kron(c_ij, eye_Hp(h,:));
                    % select only position
                    %                     q_selector = kron([1 0], eye(obj.m));
                    %                     c_i = [c_i; kron(c_ij_Hp, q_selector)];
                    c_i = [c_i; kron(kron(c_ij, eye_Hp(h,:)), kron([1 0], eye(obj.m)))];
                end
                C = [C; c_i];
            end
        end
        
        function step(obj)
           
            obj.neighbors = zeros(obj.numberAgents,1);
            % Receive messages from the network
            messages = obj.receive();
            
            obj.num_N = 0;
            % Implement the flocking protocol
            u = zeros(2, 1);
            obj.num_N = length(messages);
            C = obj.calculateC(obj.num_N);

            if obj.num_N ~= 0
                x_i = [obj.position; obj.velocity];
                xi = [x_i];
                for message = messages               
                    x_j = [message.data.position; message.data.velocity];
                    obj.neighbors(message.data.id) = 1;
                    xi = [xi; x_j];
                end            
                
                P_x = obj.calculatePx(obj.num_N);
                P_u = obj.calculatePu(obj.num_N);
                L = obj.calculateL(xi, obj.num_N);
                
                H_ = C*P_u;
                H = H_'*H_ + obj.lambda*eye(obj.m*obj.Hp);
                f = xi'*P_x'*C'*C*P_u - L'*C*P_u;
                ub = obj.u_max * ones(obj.Hp*obj.m, 1);
                lb = -1*ub;
                A = [];
                b = [];
                Aeq = [];
                beq = [];
                options = optimoptions(@quadprog, 'Display', 'off');
                x0 = zeros(obj.m*obj.Hp,1);
                
                U = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
                obj.cost = norm(C*(P_x*xi + P_u*U)-L)^2 + obj.lambda*norm(U)^2;
                
                obj.U_opt = U;
                u = U(1:obj.m);
            end
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            data.u = u;
            data.num_N = obj.num_N;
            data.id = obj.id;
            data.neighbors = obj.neighbors;
            data.U_opt = obj.U_opt;
            data.cost = obj.cost;
            obj.send(data)
        end
    end
end

