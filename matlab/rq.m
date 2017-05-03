function [R Q] = rq(M)
%	RQ - Performs a RQ decomposition
% 
% Syntax:  [R Q] = rq(M)
%
% Inputs:
%    M - Matrix to be RQ-decomposed
%
% Outputs:
%    Q - Orthogonal matrix
%    R - Upper triangular matrix
%
% Example: Intrinsics extraction from Camera matrix
%          Since P=K[R|−RC] => RQ decomposition to get K and R
% 
%   See http://cvlabwww.epfl.ch/data/multiview/fountain_dense.html for P
%   and see page 163, Section 6.2.4: Decomposition of the camera matrix in
%   R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
%   Cambridge Univ. Press, 2003. 2nd Edition
% 
%    P  = [-105.345, -3146,    -137.149, -24575.2;
%          -1154.97, -563.355,  2646.3,  -13219.2;
%          -0.887537,-0.449183,-0.102528,-9.84483];
%    [K,R]=rq(P(:,1:3))
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Solem
% email:    *unknown*
% Website:  http://ksimek.github.io/2012/08/14/decompose/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------    

%   Performs rq decomposition of matrix A
    [Q,R] = qr(flipud(M)');
    R = flipud(R');
    R = fliplr(R);

    Q = Q';   
    Q = flipud(Q);
    
end

%------------- END OF CODE --------------
