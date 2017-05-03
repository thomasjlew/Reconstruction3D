function [ X ] = cv_triangulate( v1, v2, P1, P2 )
% CV_TRIANGULATE - Triangulates the position of given matches

%   Solve using The Direct Linear Transformation (DLT) algorithm
%   R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
%   Cambridge Univ. Press, 2003. 2nd Edition: pages 312 and 91

% Syntax:  [ X ] = cv_triangulate( v1, v2, P1, P2 )
%
% Inputs:
%    v1 - (2 x N) Matched features in the first image
%    v2 - (2 x N) Matched features in the second image
%    P1 - Projection matrix associated with the first image
%    P2 - Projection matrix associated with the second image
%
% Outputs:
%    X - (4 x N) Triangulated features in homogeneous coordinates
%
% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------

x1 = int16(v1(:,1));
y1 = int16(v1(:,2));
x2 = int16(v2(:,1));
y2 = int16(v2(:,2));
X = [];                 %   3d points homogeneous coordinates
number_matches = size(v1,1);

%   for each point
for i=1:number_matches
    A = [double(x1(i)) * P1(3,:) - P1(1,:);
    	 double(y1(i)) * P1(3,:) - P1(2,:);
         double(x2(i)) * P2(3,:) - P2(1,:);
    	 double(y2(i)) * P2(3,:) - P2(2,:)];
     %  Need to perform normalisation as well.... :(
    [U,S,V] = svd(A);
    
    %   Set last coordinate (w) to 1
    V(:,4) = V(:,4) ./ V(4,4);
    
    %   Point coordinates are last column of V
    X = [X; V(:,4)'];
end

end

%------------- END OF CODE --------------
