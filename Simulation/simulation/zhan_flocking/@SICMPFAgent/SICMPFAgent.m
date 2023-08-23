%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

% This class is a implementation of the MPC flocking algorithm in
% J. Zhan and X. Li, 
% "Flocking of Multi-Agent Systems Via Model Predictive Control Based on Position-Only Measurements," 
% in IEEE Transactions on Industrial Informatics, vol. 9, no. 1, pp. 377-385, Feb. 2013, 
% doi: 10.1109/TII.2012.2216536

classdef SICMPFAgent < IntegratorAgent
    
    % Define constants of the flocking protocol
    properties
        d;          % equilibrium distance(or desired distance) to neighbours
        lambda;     % flocking weight
        Hp;         % prediction horizon
        m;          % state dimensions
        n;          % stacked state dimension
        fval;
        time;
    end
    
    % member variables
    properties
        T;          % sampling time
        p_x;        % single agent state transition matrix
        p_u;        % single agent input matrix
        numberAgents;
        adjacency;
        r;
        U_opt;       
    end
    
    methods
        X = separateState(obj);
        U_shifted = shiftU(obj, U_opt)
    end
    
    methods(Static)
        function e_nm = e_nm(n,m)
            e_nm = zeros(1,n);
            e_nm(m) = 1;
        end
        
    end
    
    methods
        function obj = SICMPFAgent(id, param, initialPos, initialVel, cfg)
            % call parent constructor
            obj@IntegratorAgent(id, param.dT, initialPos);
            
            % load data from config file
            obj.d = cfg.d;
            obj.lambda = cfg.lambda;
            obj.Hp = cfg.Hp;
            obj.m = cfg.m;
            obj.r = param.range;
            
            % initialize member variables
            obj.T = param.dT;
            obj.numberAgents = param.agentCount;
            obj.n = obj.m*obj.numberAgents;
            obj.U_opt = zeros(obj.n*obj.Hp,1);
            obj.time = 0;
            
            
            % calculate adjacency matrix
            state_s = obj.separateState();
            obj.adjacency = zeros(obj.numberAgents);
            for i  = 1:obj.numberAgents
                for j = 1:obj.numberAgents
                    if (i~=j)
                        distance = norm(state_s(:,i)-state_s(:,j));
                        if (distance <= obj.r)
                            obj.adjacency(i,j) = 1;
                        end
                    end
                end
            end
            
            % set up p_x and p_u for prediction horizon
            A = eye(obj.n);
            B = obj.T*eye(obj.n);
            p_x = [];
            for h = 1:obj.Hp
                p_x = [p_x; A^h];
            end
            obj.p_x = p_x;
            obj.p_u = kron(flip(hankel(obj.T*ones(obj.Hp,1))),eye(obj.n));
        end
        
        function step(obj)
            obj.time = obj.time + obj.T;
            fprintf("Current simulation time: %.2g\n",obj.time)
            
            state = obj.position;
            
            % states of all agents for each step of the prediction horizon
            s_hp = @(hp) kron(obj.e_nm(obj.Hp,hp),eye(obj.n)); % gives state of overall system for one step
            s_i =  @(i) kron(obj.e_nm(obj.numberAgents,i),eye(obj.m)); % gives position or velocity for one agent at one step
            
            % set up optimization
            % Z: [U], U: Hp*n x 1
            
            U0 = obj.shiftU(obj.U_opt);
            Z0 = U0;
            options = optimoptions('fmincon','Display', 'off');
            options = optimoptions(options,'Algorithm','sqp');
            [Z,obj.fval,flag,~] = fmincon(@objectiveFunction,Z0,[],[],[],[],[],[],[],options);
            if (flag == -2)
                error("No feaslible solution found.")
            end
            obj.U_opt = Z;
            u = Z(1:obj.n);
            
            function objective = objectiveFunction(Z)
                objective = 0;
                U = Z;
                X = obj.p_x*state + obj.p_u*U;
                
                % input
                objective = objective + obj.lambda*(U'*U);
                
                % flocking
                for h = 1:obj.Hp
                    x = s_hp(h)*X;
                    for i = 1:obj.numberAgents
                        qi = s_i(i)*x;
                        neighbors = find(obj.adjacency(i,:)>0.5);
                        for j = neighbors
                            qj = s_i(j)*x;
                            objective = objective + (norm(qi-qj)-obj.d)^2;
                        end
                    end
                end
            end
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            data.u = u;
            data.id = obj.id;
            data.fval = obj.fval;
            obj.send(data)
        end
    end
end

