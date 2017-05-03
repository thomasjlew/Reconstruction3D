function [proj_pts1, proj_pts2, proj_pts3] = ...
                                    project_pts(P1, P2, P3, homog_3d_pts)
% PROJECT_PTS - Computes the 2D projections of the 3D homogeneous points

% Syntax:  [proj_pts1, proj_pts2, proj_pts3] = ...
%                                     project_pts(P1, P2, P3, homog_3d_pts)
%
% Inputs:
%    P1 - First camera matrix ( P = K * [R | t] )
%    P2 - Second camera matrix
%    P3 - Third camera matrix
%
% Outputs:
%    Projected points on three camera matrices
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% May 2017; Last revision: 2-May-2017

%------------- BEGIN CODE --------------

% 1st Camera
proj_pts1 = (P1 * homog_3d_pts')';
% Normalise last component (Z = 1)
proj_pts1(:,1) = proj_pts1(:,1)./proj_pts1(:,3);
proj_pts1(:,2) = proj_pts1(:,2)./proj_pts1(:,3);
proj_pts1(:,3) = proj_pts1(:,3)./proj_pts1(:,3);

% 2nd Camera
proj_pts2 = (P2 * homog_3d_pts')';
% Normalise last component (Z = 1)
proj_pts2(:,1) = proj_pts2(:,1)./proj_pts2(:,3);
proj_pts2(:,2) = proj_pts2(:,2)./proj_pts2(:,3);
proj_pts2(:,3) = proj_pts2(:,3)./proj_pts2(:,3);

% 3rd Camera
proj_pts3 = (P3 * homog_3d_pts')';
% Normalise last component (Z = 1)
proj_pts3(:,1) = proj_pts3(:,1)./proj_pts3(:,3);
proj_pts3(:,2) = proj_pts3(:,2)./proj_pts3(:,3);
proj_pts3(:,3) = proj_pts3(:,3)./proj_pts3(:,3);

end

%------------- END OF CODE --------------
