% CHECKF - Checks the precision of the fundamental matrix F
%
%   Computes, for each pixel matches, x1_T * F * x2, which should be equal
%   to zero by definition of the fundamental matrix.
% 
% Syntax:  checkF
%
% Inputs:
%    v1 - Matched features in the first image
%    v2 - Matched features in the second image
%
% Outputs:
%    vect_errors - A vector of errors for each pair of pixels
%
% Example: 
%    save_matches                           %   Reads the matches from file
%    [ F ] = compute_fund_mat( v1, v2 )     %   Computes F from the matches
%    checkF
%
% Other m-files required: save_matches.m, compute_fund_mat.m
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------

v1;
v2;
u1 = [v1,ones(size(v1,1),1)];
u2 = [v2,ones(size(v2,1),1)];
vect_errors=[];

% F=[-2.045502044587377e-08, -4.038839632077905e-07, 0.0002453973195271189;
%   1.085739494622319e-06, -9.320244552249179e-08, 0.0104090781125352;
%   -0.001215204791423071, -0.01151842298015024, 0.9999999999999999];

for i = 1:length(v1)
    vect_errors = [vect_errors; u2(i,:) * F * u1(i,:)'];
end
vect_errors

%------------- END OF CODE --------------
