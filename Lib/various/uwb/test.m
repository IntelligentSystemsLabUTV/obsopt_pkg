close all
clear all
clc

% anchors
n = 5;
% v1 = [0,0,0];
% v2 = [1,5,0];
% v3 = [1,-5,5];
% v4 = [5,0,5];
% v5 = [15,2,5];
% 
% 
% P_r = [1.5, 1.5, 2];
% 
% P_a = zeros(3,n);

v1 = [0,0];
v2 = [1,5];
v3 = [1,-5];
v4 = [5,0];
v5 = [15,2];


P_r = [1.5, 1.5];

P_a = zeros(2,n);

for i=1:n
    eval(['P_a(:, ' num2str(i) ') = v' num2str(i) ';']); 
end
vec = zeros(1,n);

% USAGE: 
%   P_r must be a vector (1,2) [or (2,1)]
%   P_a must be a matrix (n,2) [or (2,n)]
%   bias correction: bool
%   dist_vec not use at this time -> zeros(1,n)
%   method: 0 = gradient method, else = newton (suggested)
%   epsilon: error

uwb_est(P_r, P_a, false, vec, 5, 1e-4, false)


