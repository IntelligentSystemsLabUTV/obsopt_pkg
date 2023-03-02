%%
clc
clear

A = [0  1   0;  ...
     0  0   -1; ...
     0  0   0];     

B = [0  0   1]';
C = [0  0   1];
D = 0;

sysc = ss(A,B,C,D);

Ts = 1e-2;

sysd = c2d(sysc,Ts);

syms theta [1 4]
syms s


Gamma = [-theta(1)   0           0; ...
         0          -theta(2)    0; ...
         -theta(3)  -theta(4)    0];


I = eye(3);

PHI = (I + Gamma)*sysd.A;
PHI_pol = (s*I - Gamma);

thetaval = [0.5 0.5 0 -30];

PHIval = double(subs(PHI,theta,thetaval));