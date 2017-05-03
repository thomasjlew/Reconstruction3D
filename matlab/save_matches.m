function [ v1, v2, number_matches] = compute_fund_mat(filename)
% SAVE_MATCHES - Save matches from a file into two 2xN matrices, 
%                  with N the number of matches
% 
% Syntax:  save_matches
%
% Inputs:
%    filename
%    Example:
%        matches.txt - File with matched features. Structure of the file:
% 
%             1st match: 		point1.x, point1.y;
%             2nd match: 		point1.x, point1.y;
%             3rd .....		.........
% 
%             1st match:		point2.x, point2.y;	
%             2nd match:		point2.x, point2.y;	
%             3rd .....		.........			
%
% Outputs:
%    v1             - Points in the first  image of the matched features 
%    v2             - Points in the second image of the matched features 
%    number_matches - number of matches
%
% Example: 
%   [v1,v2, number_matches] = save_matches('../build/Fund_matches.txt');
%
% Other files required: matches.txt or ../build/matches.txt
% Subfunctions: none
% MAT-files required: none

% Author:   Thomas Lew
% email:    thomas.lew@epfl.ch
% Website:  https://github.com/thomasjlew/
% April 2017; Last revision: 8-April-2017

%------------- BEGIN CODE --------------

v1 = [];
v2 = [];

fileID = fopen(filename,'r');        %   Open the file in read-only mode
% fileID = fopen('matches.txt','r');     
% fileID = fopen('Fund_matches.txt','r');      
% fileID = fopen('../build/Fund_matches.txt','r');
% fileID = fopen('../build/matches.txt','r');
formatSpec = '%f %f';           %   Format of the data in the file (float)
sizeA = [2 Inf];                        %   Size of the matrix to store dat
A = fscanf(fileID,formatSpec, sizeA);   %   Read the data in the file
A = A';                                 %   Transpose to get a vector

%   Store this data in two vectors: the matches
number_matches = size(A,1)/2;
v1 = A(1:number_matches, :)
v2 = A(number_matches+1:end, :)

end
%------------- END OF CODE --------------
