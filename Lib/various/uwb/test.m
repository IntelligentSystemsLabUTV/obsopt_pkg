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

uwb_est(P_r, P_a, false, n, vec, 0)