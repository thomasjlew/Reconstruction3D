function [ A ] = skew_sym_mat( a )
% SKEW_SYM_MAT - Computes the skew symmetric matrix of a vector
% 
% Syntax:  skew_sym_mat(a)
%
% Inputs:
%    a - Vector
%
% Outputs:
%    A - skew symmetric matrix associated to vector a
%
% Example: 
%   A = skew_sym_mat(a)
%
% Other files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------

A = [  0  , -a(3),  a(2);
      a(3),   0  , -a(1);
     -a(2),  a(1),   0   ];
     
end

%------------- END OF CODE --------------
