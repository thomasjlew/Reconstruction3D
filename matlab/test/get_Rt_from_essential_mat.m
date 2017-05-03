function [R, t] = get_Rt_from_essential_mat(E, K, pt1, pt2)
%	GET_RT_FROM_E_MATRIX - Computes the rotation and translation between
%                          two cameras from the essential matrix and the
%                          best available pair of matches.
%                          The intrinsics matrix K is used to check the 
%                          solution.
%
%   Reference:
%   Algorithm from page 259, Section 9.6 of 2nd edition of 
%   [1] R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
%   Cambridge Univ. Press, 2003.
% 
% Syntax:  [R, t] = get_Rt_from_essential_mat(E, K, pt1, pt2)
%
% Inputs:
%    E   - Essential matrix
%    K   - Intrinsics matrix
%    pt1 - Best match [1 x 2] in first  image to check [R|t] solution
%    pt2 - Best match [1 x 2] in second image to check [R|t] solution
%
% Outputs:
%    R - Rotation matrix    between first and second camera frames
%    t - translation vector between first and second camera frames
%
% Example: 
%     pt1 = v1(1,:); pt2 = v2(1,:);   % Best match to check [R|t] solution
%     [R, t] = get_Rt_from_essential_mat(E, K, pt1, pt2);
%
% Other m-files required: cv_triangulate
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 21-April-2017

%------------- BEGIN CODE --------------

%   SVD decomposition of Essential matrix to later compute the second
%   projection matrix.
[U,S,V] = svd(E);

Z = [0 1 0;
    -1 0 0;
     0 0 1];
W = [0 -1  0;
     1  0  0;
     0  0  1];
 
% Four solutions for the reconstructed projection matrix P2.
R1 = U*W*V';
t1 = U(:, 3);
R2 = U*W*V';
t2 = -U(:, 3);
R3 = U*W'*V';
t3 = U(:, 3);
R4 = U*W'*V';
t4 = -U(:, 3);

% P1 is set at the origin of the world frame, with same orientation
P1 = K*[eye(3), zeros(3,1)];

% Second camera matrix, four solutions
P2_1 = K*[R1, t1];
P2_2 = K*[R2, t2];
P2_3 = K*[R3, t3];
P2_4 = K*[R4, t4];

%   Triangulate
x1 = cv_triangulate(pt1, pt2, P1, P2_1);
x2 = cv_triangulate(pt1, pt2, P1, P2_2);
x3 = cv_triangulate(pt1, pt2, P1, P2_3);
x4 = cv_triangulate(pt1, pt2, P1, P2_4);

depth_ptx1p1 = depth_pt(x1, P1);

% Select the camera matrices such that the point is in front of the cameras
if (depth_pt(x1, P1)>0 && depth_pt(x1, P2_1)>0)
    warning('Recover [R|t]: Solution 1')
    R = R1;
    t = t1;
elseif (depth_pt(x2, P1)>0 && depth_pt(x2, P2_2)>0)
    warning('Recover [R|t]: Solution 2')
    R = R2;
    t = t2;
elseif (depth_pt(x3, P1)>0 && depth_pt(x3, P2_3)>0)
    warning('Recover [R|t]: Solution 3')
    R = R3;
    t = t3;
elseif (depth_pt(x4, P1)>0 && depth_pt(x4, P2_4)>0)
    warning('Recover [R|t]: Solution 4')
    R = R4;
    t = t4;
else
    warning('Problem: No [R|t] can reconstruct the 3d Points')
    R = zeros(3,3);
    t = zeros(3,1);
end

end

%------------- END OF CODE --------------
