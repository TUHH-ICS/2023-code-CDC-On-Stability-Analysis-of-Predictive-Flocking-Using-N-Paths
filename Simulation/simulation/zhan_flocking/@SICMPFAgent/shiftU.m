%---------------------------------------------------------------------------------------------------
% For Paper
% "On Stability Analysis of Predictive Flocking Using N-Paths"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------
function U_shifted = shiftU(obj, U_opt)
%SHIFTU shift U_opt one step in the future
% Inputs
%   U_opt   :   last step optimal input
% Outputs
%   U       :   shiftet (warm start) U_opt
U_shifted = zeros(size(U_opt));
U_shifted(1:end-obj.n) = U_opt(obj.n+1:end);
U_shifted(end-(obj.n-1):end) = zeros(obj.n,1);
end
