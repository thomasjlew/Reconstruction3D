function [ ] = visualize_3d_pts( homog_3d_pts, file_name )
% VISUALIZE_3D_PTS - Exports to a ply file and visualize 3d points
% 
% Syntax:  visualize_3d_pts( homog_3d_pts, file_name )
%
% Inputs:
%    homog_3d_pts - 3D points in homogeneous representation [N x 4]
%    file_name    - Title of graph that will be showed, name of the file to
%                   write read ply features from ("file_name".ply)
%
% Outputs:
%    Representation of 3D points
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 2-May-2017

%------------- BEGIN CODE --------------

% 3D Points
X = homog_3d_pts;

%% Save 3D points into a ply file
fileID = fopen(file_name + string('.ply'),'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
formatSpec = 'element vertex %d\n';
fprintf(fileID, formatSpec,length(X));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'element face 0\n');
fprintf(fileID, 'property list uchar int vertex_index\n');
fprintf(fileID, 'end_header\n');

for i=1:length(X)
    formatSpec = '%f %f %f\n';
    fprintf(fileID, formatSpec,X(i,1), X(i,2), X(i,3));
end

fclose(fileID);

%% Visualize the results

ptCloud = pcread(file_name + string('.ply'));

% Set the color to the same everywhere
pointscolor=uint8(zeros(ptCloud.Count,3));
pointscolor(:,1)=50;
pointscolor(:,2)=50;
pointscolor(:,3)=50;
ptCloud.Color=pointscolor;

figure
pcshow(ptCloud, 'MarkerSize', 600)

title(file_name)
xlabel('X')
ylabel('Y')
zlabel('Z')

end

