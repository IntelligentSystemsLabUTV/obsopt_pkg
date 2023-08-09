%% symbolic definition
clear
close all
clc

% symbols
q = sym('q', [4,1], 'real');
w = sym('w', [3,1], 'real');
MEAS = [0.1 0.1 0.1];
GAMMA = [0.5 0 0]';
syms R P Y real


%% function definition - continuous time
OMEGA = [...
        0       -w(3)   w(2)        w(1); ...
        w(3)    0       -w(1)       w(2); ...
        -w(2)   w(1)    0           w(3); ...
        -w(1)   -w(2)   -w(3)       0
         ];
fq = symfun(OMEGA*q, [q; w]);

% gradient
% loop - q
for i=1:4
   Gfq(:,i) = diff(fq,q(i));
end

% loop - w
for i=1:3
   Gfq(:,4+i) = diff(fq,w(i));
end

JAq = symfun(Gfq, [q;w]);

%% function definition - discrete time
% from angle to quat
Q2A = [
    atan2( 2*( q(1)*q(2) + q(3)*q(4) ), 1 - (q(2)^2 + q(3)^2)); ...
    asin( 2*( q(1)*q(3) - q(4)*q(2) )); ...
    atan2( 2*( q(1)*q(4) + q(2)*q(3) ), 1 - (q(3)^2 + q(4)^2));
    ];
fQ2A = symfun(Q2A,q);

% from quat to angle
A2Q = [
    sin(R/2); ...
    sin(P/2); ...
    sin(Y/2); ...
    cos(R/2) + cos(P/2) + cos(Y/2)
    ];
fA2Q = symfun(A2Q,[R P Y]);

% combine in jump map
TMP = (GAMMA.*MEAS + (1-GAMMA).*(fQ2A(q(1),q(2),q(3),q(4))));
JUMPq = fA2Q(TMP(1),TMP(2),TMP(3));
fJUMPq = symfun(JUMPq,q);

% gradient
% loop - q
for i=1:4
   GJUMPq(:,i) = simplify(diff(JUMPq,q(i)));
end

JJUMPq = symfun(GJUMPq, q);



