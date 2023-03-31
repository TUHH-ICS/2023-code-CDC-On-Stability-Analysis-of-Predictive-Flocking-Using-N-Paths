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
addpath(genpath('evaluation'),genpath('simulation'), genpath('data'))
dataPath = "data/";

%% Select Data to Evaluate
% videos are available in the "root/video" directory

% Available Data
simData = [
    "zhan_flocking"            % 1
    "zhang_flocking"           % 2
    ];

% Select the scenario to evaluate by setting the scenarioIndex. More data
% can be added to the comparison by changing the dataSelection arrays.

% Scenarios
% 1: Zhan Flocking
% 2: Zhang Flocking

scenarioIndex = 2;

switch (scenarioIndex)
    case (1)
        dataSelection = [1];
        for j=1:length(dataSelection)
            figure()
            xlabel('x');
            ylabel('y');
            load(dataPath+simData(dataSelection(j)));
            positions = out.data.position;
            pos_shifted = positions-positions(end,:,:);
            for i = 1:size(out.data.position,3)
                plot(pos_shifted(:,1,i),pos_shifted(:,2,i),'b'); hold on;
            end
            plot(squeeze(pos_shifted(1,1,:)),squeeze(pos_shifted(1,2,:)),'kx','MarkerSize',10,'LineWidth',1); hold on;
        end
    case (2)
        dataSelection = [2];
        for j = 1:length(dataSelection)
            load(dataPath+simData(dataSelection(j)));
            figure()
            xlabel('time');
            ylabel('J_i');
            for i = [1 2 4]
                load(dataPath+simData(dataSelection(j)));
                plot(out.t(2:end)-0.3,out.data.cost(2:end,:,i),'DisplayName',strcat("Agent ",num2str(i))); hold on;
            end
            xlim([0 3])
            ylim([0 2.5])
            legend show;
            grid on;
        end
end
