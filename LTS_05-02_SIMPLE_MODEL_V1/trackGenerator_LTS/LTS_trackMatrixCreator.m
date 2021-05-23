function M = LTS_trackMatrixCreator(name)
%% LTS_matrixCreator
% AUTHOR:
% David Izquierdo
% 
% FUNCTION: 
% Read text file containing track centreline points and order the points in a matrix
% 
% INPUTS:
% name: filename
%
% OUTPUTS:
% M: matrix of (x,y,z) points (size: N x 3)
%
% STRUCTURE:
% 1. READ FILE
% 2. MATRIX SETUP

%% 1. READ FILE
fileID = fopen(strcat(name,".txt"));
M0 = textscan(fileID,'%s','delimiter',',');

M0 = M0{1}; % everything stored by default in a "big" single cell
pattern=";";

j = 0;
Mtemp = zeros(length(M0),1);
for i = 1:length(M0)
    if contains(M0{i},pattern) % ignore the entries containing ";" (at the end of each line)
    else
        j = j+1;
        Mtemp(j) = str2double(M0{i}); % store variables in a temporal variable
    end
end

if mod(length(Mtemp),3) ~= 0 % ensure that it is divisible by 3 for the reshape to work
    Mtemp(length(Mtemp)+1) = 0;
end

%% 2. MATRIX SETUP
M = reshape(Mtemp,3,[]).';
M(~any(M,2),:) = []; % remove empty rows
M(end+1,:) = M(1,:);

