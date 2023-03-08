%%
clc
clear

%% system
A = [-0.02  -1.4    9.8;    ...
     -0.01  -0.4    0;      ...
     0      1       0];
B = [9.8 6.3 0]';
C = [1 0 0];
Ts = 3.425;%1 + 3*rand();
sysc = ss(A,B,C,0);

%% perp and P
eA = expm(A*Ts);
eA_ = expm(-A*Ts);
Cp = null(C);
P = [0.8334 -0.0041 0.8333;     ...
    -0.0041 2.0116  -0.0359;    ...
    0.8333  -0.0359 0.8341];

%% compute gain
T1 = Cp*pinv(Cp'*eA_'*P*eA_*Cp)*Cp';
T2 = (eA_'*P*eA_)'*C';
K = (C'-T1*T2)*pinv(C*C');

%% dynamics matrix
PHI = (eye(3)-K*C)*eA;
eig(PHI)