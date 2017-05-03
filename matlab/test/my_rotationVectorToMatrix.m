function [ rotationMatrix ] = my_rotationVectorToMatrix( w, theta )
% MY_ROTATIONVECTORTOMATRIX - Computes the rotation matrix from a rotation 
%                             vector using Rodriguez formula
% Reference:
%       https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
%       Note: this function is already implemented in Matlab2017a

% Assumptions:
%       - the angular velocity is normalised.
%       - angle in radians

% Requires the computation of a skew symmetric matrix, computed here with
%       skew_sym_mat( a )= [  0  , -a(3),  a(2);
%                             a(3),   0  , -a(1);
%                            -a(2),  a(1),   0   ]

% Syntax:  [R, t] = get_Rt_from_essential_mat(E, K, pt1, pt2)
%
% Inputs:
%    w     - Normalized rotation vector, indicates the rotation direction
%    theta - Angle of rotation
%
% Outputs:
%    rotationMatrix - Rotation matrix around vector 'w' of angle 'theta'
%
% Example: 
%     R = my_rotationVectorToMatrix( [0;1;0], pi/4 );

% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 10-April-2017

%------------- BEGIN CODE --------------

w_hat = [  0  , -w(3),  w(2);
          w(3),   0  , -w(1);
         -w(2),  w(1),   0   ];

rotationMatrix = eye(3) + sin(theta) * w_hat + ...
                    (1-cos(theta)) * w_hat * w_hat;

end

%------------- END OF CODE --------------
