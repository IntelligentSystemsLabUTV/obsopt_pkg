
%% symbolic definition
% clear
% close all
% clc

% symbols
q = sym('q', [4,1], 'real');
w = sym('w', [3,1], 'real');
MEAS = sym('M', [3,1], 'real');
GAMMA = sym('GAMMA', [3,1], 'real');
THETA = sym('THETA', [3,1], 'real');
syms R P Y real


%% function definition - continuous time
OMEGA = [...
        0       -w(3)   w(2)        w(1); ...
        w(3)    0       -w(1)       w(2); ...
        -w(2)   w(1)    0           w(3); ...
        -w(1)   -w(2)   -w(3)       0
         ];
fq = symfun(0.5*OMEGA*q, [q; w]);
fq = [fq;diag(w)*zeros(3,1)];

% gradient
% loop - q
for i=1:4
   Gfq(:,i) = diff(fq,q(i));
end

% loop - w
for i=1:3
   Gfq(:,4+i) = diff(fq,w(i));
end

fAqbar = symfun(Gfq, [q;w]);


%% function definition - discrete time
% from angle to quat
Q2A = [
    atan2( 2*( q(1)*q(4) + q(2)*q(3) ), 1 - (q(1)^2 + q(2)^2)); ...
    asin( 2*( q(4)*q(2) - q(1)*q(3) )); ...
    atan2( 2*( q(3)*q(4) + q(1)*q(2) ), 1 - (q(2)^2 + q(3)^2));
    ];
fQ2A = symfun(Q2A,q);

% from quat to angle
A2Q = [
    sin(R/2); ...
    sin(P/2); ...
    sin(Y/2); ...
    cos(R/2) + cos(P/2) + cos(Y/2); ...
    ];
fA2Q = symfun(A2Q,[R P Y]);

% Lnearization of Q2A and A2Q
% loop on RPY
A2Qbar(:,1) = simplify(diff(fA2Q,R)); 
A2Qbar(:,2) = simplify(diff(fA2Q,P)); 
A2Qbar(:,3) = simplify(diff(fA2Q,Y)); 
fA2Qbar = symfun(A2Qbar,[R P Y]);

% loop - q
for i=1:4
   Q2Abar(:,i) = simplify(diff(fQ2A,q(i))); 
end

fQ2Abar = symfun(Q2Abar,q);

%% function definition - position

% continuous time
Ae = zeros(9);
Ae(1:3,4:6) = +eye(3);
Ae(4:6,7:9) = -eye(3);
fAe = Ae;

% discrete time
Aed = THETA(1)*zeros(9);
Aed(1:3,1:3) = THETA(1)*eye(3);
Aed(4:6,1:3) = THETA(2)*eye(3);
Aed(7:9,1:3) = THETA(3)*eye(3);
fAed = symfun(Aed,THETA);

