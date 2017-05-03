function [ F, err_F ] = compute_fund_mat( v1, v2 )
%	COMPUTE_FUND_MAT - Computes the fundamental matrix F from corresponding
%                      pixel matches
%
%   Performs least-squares minimization to get the fundamental matrix
%   which satisfies x2' * F * x1 = 0, using normalised 8 point algorithm
%
%   Algorithm from page 282, Chapter 11 of 2nd edition of 
%   R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
%   Cambridge Univ. Press, 2003. [1]
% 
% Syntax:  [ F ] = compute_fund_mat( v1, v2 )
%
% Inputs:
%    v1 - Matched features in the first image  [Nx2]
%    v2 - Matched features in the second image [Nx2]
%
% Outputs:
%    F - Fundamental matrix
%
% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 7-April-2017

%------------- BEGIN CODE --------------

%   Check if the matched features have the same length
if (length(v1) ~= length(v2))
    error('Error computing fundamental matrix: features dont match');
    F = [];
end
%   Check if the matched features are enough to compute F
if (length(v1) < 8)
    error('Error computing fundamental matrix: not enough features');
    F = [];
end

%   Normalise data
%   http://cs.adelaide.edu.au/~wojtek/papers/pami-nals2.pdf
%   IV.  HARTLEY’S APPROACH
%   Difference: the average distance is sqrt(2) instead of root avrg. dist.
%   as described in [1]

%   Compute centroids ("center of masses")
m1 = [mean(v1(:,1)), mean(v1(:,2))];
m2 = [mean(v2(:,1)), mean(v2(:,2))];

%   Compute current average distance to center
norms1 = []; norms2 = [];
for i=1:length(v1)
    norms1 = [norms1; norm(v1(i,:) - m1)];
    norms2 = [norms2; norm(v2(i,:) - m2)];
end
s1 = mean(norms1);
s2 = mean(norms2);

%   Normalise data: center it and scale its average distance to center of
%   centroid to be equal to one (could be sqrt(2) as in HZ book)
v1 = (v1 - ones(length(v1),1)*m1) * (sqrt(2)/s1);
v2 = (v2 - ones(length(v2),1)*m2) * (sqrt(2)/s2);

%   Compute the inverse transformation (to get F) 
T1 = [sqrt(2)/s1,          0, -m1(1)*sqrt(2)/s1;
               0, sqrt(2)/s1, -m1(2)*sqrt(2)/s1;
               0,          0,                 1];
T2 = [sqrt(2)/s2,          0, -m2(1)*sqrt(2)/s2;
               0, sqrt(2)/s2, -m2(2)*sqrt(2)/s2;
               0,          0,                 1];

  
%   Build matrix A from the matches
A = [];
for i=1:length(v1)
    x1 = v1(i,1); y1 = v1(i,2);
    x2 = v2(i,1); y2 = v2(i,2);
    a = [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1];
    
    A = [A; a];
end

%   Perform Singular value decomposition
[U, D, V] = svd(A);

f = V(:,end);

%   Recompose matrix F~norm(v2 - ones(length(v2),1) * m2);
F = [f(1), f(2), f(3);
     f(4), f(5), f(6);
     f(7), f(8), f(9)];

%   Enforce singularity (Constraint enforcement) ( rank(F)=2 )
U = [];
D = [];
V = [];
[U, D, V] = svd(F);
D(3,3) = 0;         %   Enforce the smallest eigenvalue to be zero

%   Reform F which is now rank 2
F = U*diag(diag(D))*V';

%   Undo normalisation to get the fundamental matrix
F = T2' * F * T1;

%   Rescale to have F(3,3)=1
F = F / F(3,3);

%   Compute the error with this fundamental matrix
err_F = 0;
for i = 1:length(v1)
    err_F = err_F + abs([v2(i,:),1] * F * [v1(i,:),1]');
end

end

%------------- END OF CODE --------------