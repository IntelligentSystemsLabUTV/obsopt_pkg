%% just to be quick - load data and align signals

% A(1) --> vicon
% A(2) --> UWB
% A(3) --> EKF
% A(4) --> PJUMP
% A(5) --> IMU

% stop time
Tf = [];
data = [];
out = [];

close all

%% LEARN

% Lab02
% out(1).val=readBag('data/jackal/lab/Lab02.bag');
% A = 1*[-25/0.01, 0, 7.5/0.01, 3.5/0.2, 11/0.01];
% data(1).val=plotBag(out(1).val,1,1,Tf,0.01,A);

% Lab03
% out(2).val=readBag('data/jackal/lab/Lab03.bag');
% A = 1*[-25/0.01, 0, 6.5/0.01, 10.5/0.2, 10.5/0.01];
% data(2).val=plotBag(out(2).val,1,1,Tf,0.01,A);

%% TEST

% Lab01
% out(1).val=readBag('data/jackal/lab/Lab01.bag');
% A = 1*[-24/0.01, 0, 7.5/0.01, 2/0.2, 11/0.01];
% data(1).val=plotBag(out(1).val,1,1,Tf,0.01,A);

% Lab04
% out(1).val=readBag('data/jackal/lab/Lab04.bag');
% A = 1*[-35/0.01, 0, 2.5/0.01, 2.5/0.2, 3/0.01];
% data(1).val=plotBag(out(1).val,1,1,Tf,0.01,A);


%% Hangar
out(1).val=readBag('data/jackal/hangar/Hangar02.bag');
A = 1*[0, 0, 0/0.01, 1.5/0.2, 2.5/0.01];
data(1).val=plotBag(out(1).val,1,0,Tf,0.01,A);

%% tidy up
clear Tf A