%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear all;
close all;

%% Select Algorithm and Scenario
% Algorithms
% 1: Zhan CMPF
% 2: Zhang DMPF

algorithmIndex  = 2;

%% Setup
addpath(genpath('simulation/mas-simulation/lib'))
addpath(genpath('simulation'))
simPath = "simulation/";

algorithms = simPath + [...
    "zhan_flocking"
    "zhang_flocking"
    ];

preallocate = {...
    @preallocateSICMPF
    @preallocateZhang
    };

generateSetup = {...
    @generateSetupSICMPF
    @generateSetupZhang
    };
%% Set Agent Parameters
cfg = generateConfig(algorithms, algorithmIndex);

%% Simulation Setup
% define output and initialization files and paths
outPath = strcat(simPath,"out/",erase(algorithms(algorithmIndex),simPath),"/");
outFile = outPath+"results.mat";


% simulation parameters
switch algorithmIndex
    case 1
        initializationFile = simPath+"initialStates_Zhan.mat";
        Tf               = 10;  % Simulation duration [s]
        param.agentCount = 7;   % Number of agents in the network
        param.dimension  = 2;    % Dimension of the space the agents move in
        param.dT         = 0.2;  % Size of the simulation time steps [s]
        param.range      = 8.4;  % Agent interaction range
    case 2
        initializationFile = simPath+"initialStates_Zhang.mat";
        Tf               = 20;  % Simulation duration [s]
        param.agentCount = 7;   % Number of agents in the network
        param.dimension  = 2;    % Dimension of the space the agents move in
        param.dT         = 0.2;  % Size of the simulation time steps [s]
        param.range      = 8.4;  % Agent interaction range
end

init = load(initializationFile);
setup = generateSetup{algorithmIndex}(cfg, param, init);

%% Run Simulation
sim = SimulationManager(setup.Network, setup.Agents);
leech = preallocate{algorithmIndex}(setup, sim, Tf);
data = performSimulation(sim, leech, Tf);
out.t = data.t;
out.data = data.data;

% Reshape data for centralized algorithm
if algorithmIndex == 1
   % position
   out.data.position = reshape(out.data.position,size(out.data.position,1),param.dimension,[]);
   % velocity
   out.data.velocity = reshape(out.data.velocity,size(out.data.velocity,1),param.dimension,[]);
   % u
   out.data.u = reshape(out.data.u,size(out.data.u,1),param.dimension,[]);
end

save(outFile,'out','setup','param', 'cfg');