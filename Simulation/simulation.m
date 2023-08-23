%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear; close all;

%% Setup
addpath(genpath('simulation'), genpath('mas-simulation'))
simPath = "simulation/";
algorithm = simPath + "zhan_flocking";

%% Set Agent Parameters
cfg.d       = 7;    % desired inter-agent distance
cfg.m       = 2;    % space dimension
cfg.Hp      = 5;    % prediction horizon
cfg.lambda = 0.1;   % input weight

%% Simulation Setup
% define output and initialization files and paths
outPath = strcat(simPath,"out/",erase(algorithm,simPath),"/");
outFile = outPath+"results.mat";
initializationFile  = simPath+"initialStates_Zhan.mat";

% simulation parameters
Tf                  = 10;   % Simulation duration [s]
param.agentCount    = 7;    % Number of agents in the network
param.dimension     = 2;    % Dimension of the space the agents move in
param.dT            = 0.2;  % Size of the simulation time steps [s]
param.range         = 8.4;  % Agent interaction range

% load initial states
init = load(initializationFile);

%% Run Simulation
setup = generateSetupSICMPF(cfg, param, init);
sim = SimulationManager(setup.Network, setup.Agents);
leech = preallocateSICMPF(setup, sim, Tf);
data = performSimulation(sim, leech, Tf);
out.t = data.t;
out.data = data.data;

%% Save Data
% Reshape data
% position
out.data.position = reshape(out.data.position,size(out.data.position,1),param.dimension,[]);
% velocity
out.data.velocity = reshape(out.data.velocity,size(out.data.velocity,1),param.dimension,[]);
% u
out.data.u = reshape(out.data.u,size(out.data.u,1),param.dimension,[]);

% Save
save(outFile,'out','setup','param', 'cfg');