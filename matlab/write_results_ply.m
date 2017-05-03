function [ ] = write_results_ply(X)
% WRITE_RESULTS_PLY - Writes the triangulated features into a PLY file
%                     and displays the results
% 
% Syntax:  write_results_ply
%
% Inputs:
%    X - Triangulated features ( Nx4 or Nx3 )
%
% Outputs:
%    matlab_triangl_results.ply - PLY file with triangulated features
%
% Example: 
%    X = cv_triangulate(v1,v2,P1,P2)        %   Triangulation
%    write_results_ply(X)                   %   Write results into file

% Other m-files required: cv_triangulate.m
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------

figure(2)

fileID = fopen('matlab_triangl_results.ply','w');
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


ptCloud = pcread('matlab_triangl_results.ply');

pointscolor=uint8(zeros(ptCloud.Count,3));
pointscolor(:,1)=50;
pointscolor(:,2)=50;
pointscolor(:,3)=50;
ptCloud.Color=pointscolor;

pcshow(ptCloud, 'MarkerSize', 10, 'VerticalAxis', 'Y')
xlim([-10 10])
ylim([-10 10])
zlim([0 10])

title('matlab fully triangulated results')
xlabel('X')
ylabel('Y')
zlabel('Z')

end
%------------- END OF CODE --------------