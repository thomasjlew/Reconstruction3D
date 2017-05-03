% DISPLAY_CPP_RESULT_PLY - Displays triangulated featres from a PLY file
% 
% Syntax:  display_cpp_result_ply
%
% Inputs:
%    file.ply - Triangulated features file in PLY format
%               in this case, we use ../build/reconstruct.ply which is
%               created by the cpp code
%
% Outputs:
%    Triangulated features visualization
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------
close all;

figure
ptCloud = pcread('../build/reconstruct.ply')
% ptCloud = pcread('../build/reconstructCUSTOM.ply');

pointscolor=uint8(zeros(ptCloud.Count,3));
pointscolor(1:end,1)=50;
pointscolor(1:end,2)=50;
pointscolor(1:end,3)=50;
% pointscolor(1:2500,1)=50;
% pointscolor(1:2500,2)=50;
% pointscolor(1:2500,3)=50;
% pointscolor(2501:5000,1)=0;
% pointscolor(2501:5000,2)=200;
% pointscolor(2501:5000,3)=0;
pointscolor(2501:end,1)=0;
pointscolor(2501:end,2)=200;
pointscolor(2501:end,3)=0;
% pointscolor(5001:end,1)=0;
% pointscolor(5001:end,2)=0;
% pointscolor(5001:end,3)=200;
ptCloud.Color=pointscolor;

pcshow(ptCloud, 'MarkerSize', 11)
title('Triangulated features from cpp file')
xlabel('X')
ylabel('Y')
zlabel('Z')
% xlim([-10 10])
% ylim([-10 10])
% zlim([-10 10])
xlim([-10 10])
ylim([-10 10])
zlim([0 10])
% xlim([-100 100])
% ylim([-100 100])
% zlim([-100 100])

%------------- END OF CODE --------------