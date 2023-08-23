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
addpath(genpath('evaluation'),genpath('simulation'), genpath('data'))
dataPath = "data/";

%% Select Data to Evaluate

% Available Data
% additional data can be added to the comparison by changing the 
% dataSelection array and adding the name of the data to the simData array.
simData = [
    "zhan_flocking"            % 1
    ];

dataSelection = [1];

% plot agent trajectories with final state shifted to the origin
for j=1:length(dataSelection)
    figure()
    xlabel('x');
    ylabel('y');
    load(dataPath+simData(dataSelection(j)));

    % shift final state to origin
    positions = out.data.position;
    pos_shifted = positions-positions(end,:,:);

    % plot
    for i = 1:size(out.data.position,3)
        plot(pos_shifted(:,1,i),pos_shifted(:,2,i),'b'); hold on;
    end
    plot(squeeze(pos_shifted(1,1,:)),squeeze(pos_shifted(1,2,:)),'kx','MarkerSize',10,'LineWidth',1); hold on;
end

