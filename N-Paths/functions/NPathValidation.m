%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [T_B, isFeasible] = NPathValidation(T_A)
%NPATHVALIDATION generate N-path according to Lemma 1 
% Inputs
%   T_A                 :   input N-paths
%
% Outputs
%   T_B                 :   output T_B N-path according to Lemma 1
%   isFeasible          :   true if feasible T_B can be found

d = size(T_A,1);        % path dimension
N = size(T_A,2);        % path length
T_B0 = zeros(N*d + N-1,1);    % initial guess for T_B

% set B point bounds
lb = [-inf*ones(N*d,1); zeros(N-1,1)];
ub = [inf*ones(N*d,1); ones(N-1,1)];

options = optimoptions('fmincon','Display', 'off');
[X,~,exitflag,~] = fmincon(@objectiveFunction,T_B0,[],[],[],[],lb,ub,@nlcon, options);
T_B = reshape(X(1:d*N),d,[]);
if exitflag >= 0
    isFeasible = 1;
else
    isFeasible = 0;
end
% specify cost function (try to find T_B with smallest distance to origin)
    function [cost] = objectiveFunction(X)
        TB = reshape(X(1:d*N),d,[]);
        cost = 0;
        for j = 1:N
            cost = cost + norm(TB(:,j))^2;
        end
    end

% model lemma constriants
    function [c, ceq] = nlcon(X)
        TB = reshape(X(1:d*N),d,[]);
        alpha = X(d*N+1:end);
        ceq = [TB(:,1)-T_A(:,1)]; % identical first points
        c = [];
        % pointing towards O constraint
        for i = 1:N-1
            % on line from A1 to O
            ceq = [ceq; T_A(:,1)*alpha(i)-TB(:,i+1)];
            
            % decreasing
            if i > 1
                c = [c; alpha(i)-alpha(i-1)];
            end
        end
        
        for i = 1:N % first constraint
            AiO(i) = norm(T_A(:,i));
            c = [c; norm(TB(:,i)) - AiO(i)];
        end
        for i = 1:N-1 % second constraint
            AiAip(i) = norm(T_A(:,i+1)-T_A(:,i));
            c = [c; norm(TB(:,i+1)-TB(:,i)) - AiAip(i)];
        end
        
        % fourth constraint
        ceq = [ceq; TB(:,end)-TB(:,end-1) - (TB(:,end-1)-TB(:,end-2))];
        % third constraint
        for i = 1:N-2
            dAiAip(i) = norm( (T_A(:,i+2)-T_A(:,i+1)) - (T_A(:,i+1)-T_A(:,i)) );
            c = [c; norm( (TB(:,i+2)-TB(:,i+1)) - (TB(:,i+1)-TB(:,i)) ) - dAiAip(i)];
        end
    end

end

