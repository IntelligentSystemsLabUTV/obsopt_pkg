%%
clc
clear

A = [0 1 0 0; ...
0 0 1 0; ...
0 0 0 0; ...
0 0 0 0];

B = [0 0 1 0]';

C = [1 0 0 0; ...
0 0 1 1];

D = 0;

sysc = ss(A,B,C,D);

Ts = 1e-2;

sysd = c2d(sysc,Ts);

syms theta [1 8]
syms s


Gamma = [theta(1)   0           0           0; ...
         0          theta(2)    0           0; ...
         theta(3)   0           theta(4)    theta(4); ...
         theta(5)   0           0           theta(6)];


I = eye(4);

PHI = (I - Gamma);%*sysd.A;
PHI_pol = (s*I - Gamma);

thetaval = [1.2 1.7 5 5 5 0.2 5 0.9];

PHIval = double(subs(PHI,theta,thetaval));