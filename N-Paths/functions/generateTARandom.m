%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function T_A = generateTARandom(N, range, samples, dimension)
%GENERATETARANDOM generate random N-path with specified length and
% dimension.
% Inputs
%   N           : length of path
%   range       : interval of path points
%   samples     : number of paths
%   dimension   : dimension of path points
%
% Outputs
%   T_A         : 3-dimensional array matrix containing N-path points

T_A = zeros(dimension,N,samples);
midpoint = (range(1)+range(2))/2;
size = -range(1)+range(2);
for i = 1:samples
    for j = 1:N
        T_A(:,j,i) =  midpoint + size * (rand(dimension, 1) - 0.5);
        T_A(:,j,i) = round(T_A(:,j,i),1);
    end
end
end

