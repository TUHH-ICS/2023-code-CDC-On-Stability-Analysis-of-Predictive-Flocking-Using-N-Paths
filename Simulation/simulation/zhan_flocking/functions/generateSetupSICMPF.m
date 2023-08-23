%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function setup = generateSetupSICMPF(cfg, param, init)
setup = struct;
Network = IdealNetwork(1, param.dT, param.agentCount*param.dimension, param.range);
setup.Network = Network;
Agents = cell(1, 1);
Agents{1} = SICMPFAgent(Network.getId(), param, reshape(init.pos,[],1), reshape(init.vel,[],1), cfg);
setup.Agents = [Agents{:}];
end