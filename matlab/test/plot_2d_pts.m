function [ ] = plot_2d_pts(proj_pts1,proj_pts2, proj_pts3)
% PLOT_2D_PTS - Plots a set of 2d points on three seperate images
% 
% Syntax:  plot_2d_pts(proj_pts1,proj_pts2, proj_pts3);
%
% Inputs:
%    proj_pts1 - First set of 2d points [N x 2]
%
% Outputs:
%    Image with three sets of 2d points
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% May 2017; Last revision: 2-May-2017

%------------- BEGIN CODE --------------

figure

subplot(1,3,1) 
plot(proj_pts1(:,1) ,proj_pts1(:,2),'.')
set(gca,'Ydir','reverse')
title('Image 1') 

subplot(1,3,2) 
plot(proj_pts2(:,1) ,proj_pts2(:,2),'.')
set(gca,'Ydir','reverse')
title('Image 2')

subplot(1,3,3) 
plot(proj_pts3(:,1) ,proj_pts3(:,2),'.')
set(gca,'Ydir','reverse')
title('Image 3')

end

