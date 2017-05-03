%   Save Fundamental matrix

err('SAVE_FUNDMAT DOESNT WORK WITH NUMBERs LIKE E-01')
fileID = fopen('fundMat.txt','r');      %   Open the file in read-only mode
formatSpec = '%lf %lf %lf';           %   Format of the data in the file (float)
sizeF = [3 3];                        %   Size of the matrix to store dat
F = fscanf(fileID,formatSpec, sizeF)   %   Read the data in the file
% F = F'   (AXEs???)


