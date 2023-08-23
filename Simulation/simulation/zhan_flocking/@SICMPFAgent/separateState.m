%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------
function X = separateState(obj)
dimension = obj.m;
pos = obj.position;
X = reshape(pos,dimension,[]);
end

