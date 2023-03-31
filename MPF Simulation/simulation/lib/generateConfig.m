%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function cfg = generateConfig(algorithms, index)

% Algorithms
% 1: Zhan Flocking
% 2: Zhang Flocking

switch(index)
    % 1: Zhan Flocking Parameters
    case (1)
        cfg.d       = 7;    % desired inter-agent distance
        cfg.m       = 2;    % space dimension
        cfg.Hp      = 5;    % prediction horizon
        cfg.lambda = 0.1;
        
    % 2: Zhang Flocking Parameters
    case (2)
        cfg.d       = 7;    % desired inter-agent distance
        cfg.m       = 2;    % space dimension
        cfg.Hp      = 10;    % prediction horizon
        cfg.u_max   = 1;    % input constraint
        cfg.lambda  = 0.1;
        
end
save(strcat(algorithms(index),'/cfg/config.mat'),'cfg');
end

