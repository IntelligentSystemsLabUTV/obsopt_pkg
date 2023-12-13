%%
clear
clc

%% continuous time - observer
A = [0  1   0   0;  ...
     0  0   -1   1;  ...
     0  0   0   0;  ...
     0  0   0   -1];

B = [0  0   0   1]';
C = [1  0   0   0;  ...
     0  0   0   1];
D = 0;

sys_c = ss(A,B,C,D);
O = obsv(sys_c);

%% continuous time - observer correction
theta = [0.5 0 0.5 0];
A = [1-theta(1)     1   0           0;  ...
     0              0   -1          1;  ...
     1-theta(3)     0   theta(4)    0;  ...
     0              0   0           -1];

B = [theta(1)   0   theta(3)    0;    ...
     0          0   0           1]'; 
C = [1  0   0   0;  ...
     0  0   0   1];
D = 0;

sys_cc = ss(A,B,C,D);

