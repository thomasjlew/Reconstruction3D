% RECONSTRUCT - Reconstruct 'fountain_dense' using extracted features from
%               cpp code available at 
%               https://github.com/thomasjlew/Reconstruction3D
%               The features are written in 'build/Fund_matches.txt' and
%               '../build/matches.txt' when executing "cvision".
% 
% Reference: Dataset available at (Last visited 2 May 2017)
%               http://cvlabwww.epfl.ch/data/multiview/fountain_dense.html
% 
% Syntax:  reconstruct
%
% Inputs:
%    ../build/Fund_matches.txt - Precise extracted matched features file
%    ../build/matches.txt      - Extracted matched features file
%
% Outputs:
%    Vizualization - 3D reconstructed points of features
%
% Other m-files required: - save_matches
%                         - compute_fund_mat
%                         - get_Rt_from_essential_mat
%                         - cv_triangulate
%                         - write_results_ply
%                         - display_cpp_result_ply
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 2-May-2017

%------------- BEGIN CODE --------------

clear all;
close all;

addpath('./test')

%   Display the image
img = imread('../fountain_dense/urd/0007.png');
% imshow(img)
%   Size of the image used here: 3072 x 2048 pixels

%   Read the matches saved on a file
[F_v1, F_v2, F_number_matches] = save_matches('../build/Fund_matches.txt');
[v1, v2, number_matches] = save_matches('../build/matches.txt');
matches=[v1,v2];

%   X and Y axis are weird with matlab images!! => changes in K intrinsics
%           o----> X-axis           o----> X-axis
%   MATLAB: |               matches |
%           V Y-axis                V Y-axis 
%   Draw the matches on the image
ft_img = img;
x_id = int16(F_v1(:,1));
y_id = int16(F_v1(:,2));      
for i=1:F_number_matches
    ft_img(y_id(i)-15 : y_id(i)+15, x_id(i)-15 : x_id(i)+15, 1) = 0;
    ft_img(y_id(i)-15 : y_id(i)+15, x_id(i)-15 : x_id(i)+15, 2) = i;
        %   changes the colour and add the identifier as 2nd pixel value
end

%   make the first 2 matches white to check
ft_img(y_id(1)-15 : y_id(1)+15, x_id(1)-15 : x_id(1)+15, 1) = 255;
ft_img(y_id(1)-15 : y_id(1)+15, x_id(1)-15 : x_id(1)+15, 2) = 255;
ft_img(y_id(1)-15 : y_id(1)+15, x_id(1)-15 : x_id(1)+15, 3) = 100;
ft_img(y_id(2)-15 : y_id(2)+15, x_id(2)-15 : x_id(2)+15, 1) = 255;
ft_img(y_id(2)-15 : y_id(2)+15, x_id(2)-15 : x_id(2)+15, 2) = 255;
ft_img(y_id(2)-15 : y_id(2)+15, x_id(2)-15 : x_id(2)+15, 3) = 200;

% figure(1)
% h = imshow(ft_img);
% title('image with features');
% hp = impixelinfo;
% set(hp,'Position',[5 1 300 20]);
% impixelinfo;

%   To extract intrinsics, the following reference can be useful:
%   http://ksimek.github.io/2012/08/14/decompose/

%   Intrinsics
K = [-2759.48,    0,    1520.69;
	    0,    -2764.16, 1006.81;
	    0, 	     0, 	  1   ]
    
%   Projection matrix from fountain_dense dataset
%  P1 = [-105.345,    -3146, -137.149, -24575.2;
%        -1154.97, -563.355,   2646.3, -13219.2;
%       -0.887537,-0.449183,-0.102528, -9.84483]
%   Since P=K[R|−RC] => RQ decomposition
% %   [K,R]=rq(P1(:,1:3))   %   Four solutions
% %   R(1,:) = -R(1,:);
% %   R(2,:) = -R(2,:);
% %   det(R)
% %   R = -R;
% %   det(R)
% %   t = (-K*R)\P1(:,4)
    
%   Fundamental matrix computation
[ F, err_F ] = compute_fund_mat( F_v1, F_v2 )
rank(F);    % Rank of fundamental matrix should be equal to 2.

%	Computation of the Essential matrix
E = K' * F * K;

% SVD decomposition of Essential matrix to compute the Extrinsics matrix
pt1 = F_v1(1,:); pt2 = F_v2(1,:);   % Best match to check [R|t] solution
[R, t] = get_Rt_from_essential_mat(E, K, pt1, pt2)

% Form camera matrices from intrinsics and extrinsics
P1 = K*[eye(3), zeros(3,1)]
P2 = K*[R, t]

%   Triangulate
X = cv_triangulate(v1, v2, P1, P2);

%   Write into PLY file and show the results
write_results_ply(X)

% Compare with reconstruction from C++ code
display_cpp_result_ply

%------------- END OF CODE --------------