function [ depth ] = depth_pt(pt_3d, P)
%	DEPTH_PT - Computes the depth of a 3D point given the camera matrix.
%              Used to determine whether the point is in front of the
%              camera image plane or not.
%
%   References:
%       Depth of a point: Equation from page 162, Section 6.2.3 
%       Camera center   : Equations from page 163, Section 6.2.4 
%           2nd edition of 
%           [1] R. Hartley and A. Zisserman, Multiple View Geometry in
%           Computer Vision, Cambridge Univ. Press, 2003.
%       Adapted from by Diego Cheda, whose code is available at 
%           https://www.mathworks.com/matlabcentral/
%           fileexchange/47032-camera-geometry-algorithms
% 
% Syntax:  [ depth ] = depth_pt( pt_3d, P )
%
% Inputs:
%    pt_3d - 3D point in homogeneous coordinates (x,y,z,w=1) [1 x 4]
%    P   - Camera matrix P = K[R|t]
%
% Outputs:
%    depth - depth of the point to determine whether it is in front of the
%            camera or not
%
% Example: 
%     [ depth ] = depth_pt( pt_3d, P )
%
% Other m-files required: rq.m
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 21-April-2017

%------------- BEGIN CODE --------------

% Compute camera center c from camera matrix
x =  det([ P(:,2), P(:,3), P(:,4) ]);
y = -det([ P(:,1), P(:,3), P(:,4) ]);
z =  det([ P(:,1), P(:,2), P(:,4) ]);
t = -det([ P(:,1), P(:,2), P(:,3) ]);
c = [ x/t; y/t; z/t ];

M = P(:, 1:3);

% Projection of the 3d point onto the z-axis of the camera
pt_3d = pt_3d';
w = M(3,:) * (pt_3d(1:3,1) - c(1:3,1));
% norm(M(3,:));

% The 3rd row of P points towards the positive z-axis if the determinant of
% P(1,3)>0. Hence, the scalar product of w with this direction vector
% represents the depth of the 3d point
depth = sign(det(M)) * w / norm(M(3,:));
    
end

%------------- END OF CODE --------------