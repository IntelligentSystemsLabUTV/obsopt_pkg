%%
clear
clc

%% continuous time
A = [0  1   0   0;  ...
     0  0   1   0;  ...
     0  0   0   0;  ...
     0  0   0   0];

B = [0  0   0   0]';
C = [1  0   0   0;  ...
     0  0   1   1];
D = 0;

sys_c = ss(A,B,C,D);
O = obsv(sys_c);
