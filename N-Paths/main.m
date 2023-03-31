%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear; close all;
addpath(genpath('functions'))
%% Setup
nSample = 100;          % number of N-paths to be tested
interval = [-1 2];      % range of T_A points (equal for each dimension; T_A points are rounded to one decimal digit)
pathLength = 3;         % length of N-paths
dimension = 1;          % dimension of underlying space

%% Prepare output vector
infeasible = zeros(1,nSample);

%% Generate T_A Path
T_A = generateTARandom(pathLength, interval, nSample, dimension);

%% Try to find T_B
% use "parfor" instead of "for" for parallel computation
parfor i = 1:nSample
    [~, isFeasible] = NPathValidation(T_A(:,:,i));
    if ~isFeasible
        infeasible(i) = 1;
    end
end

%% Evaluation
indices = find(infeasible>0.5);
infeasiblePaths = squeeze(T_A(:,:,indices));
fprintf('%d of %d paths were feasible (%d infeasible)\n',nSample-length(indices),nSample,length(indices))

%% Save infeasible paths
outFile = "examples/" + "length-" + pathLength + "_dimension-" + dimension + ".mat";
save(outFile,'infeasiblePaths');
