function [ homog_3d_pts ] = init3dpts(  )
%	INIT3DPTS - Initialize the 3D points for TestFile.m. Generates a simple
%	model which can be reconstructed.
% 
% Syntax:  homog_3d_pts = init3dpts(  )
%
% Inputs:
%    None.
%
% Outputs:
%    homog_3d_pts - Model 3D points in homogeneous representation [13 x 4]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 2-May-2017

%------------- BEGIN CODE --------------

% 3D points (Ground truth)
pt0 = [0    ;    0;10000];
pt1 = [0    ;    0;7500];
pt2 = [ 2500;    0;7500];
pt3 = [-2500;    0;7500];
pt4 = [0    ; 2500;7500];
pt5 = [0    ;-2500;7500];
pt6 = [ 2500; 2500;7500];
pt7 = [ 2500;-2500;7500];
pt8 = [-2500; 2500;7500];
pt9 = [-2500;-2500;7500];
pt10 = [-2500;    0;10000];
pt11 = [-2500;-2500;10000];
pt12 = [-2500; 2500;10000];

% Homogeneous representation (4 coordinates: X,Y,Z,W)
homog_3d_pts = [pt0', 1;
                pt1', 1;
                pt2', 1;
                pt3', 1;
                pt4', 1;
                pt5', 1;
                pt6', 1;
                pt7', 1;
                pt8', 1;
                pt8', 1;
                pt9', 1;
                pt10', 1;
                pt11', 1;
                pt12', 1;];
end

%------------- END OF CODE --------------
