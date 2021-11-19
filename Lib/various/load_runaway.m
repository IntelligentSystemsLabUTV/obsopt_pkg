%% load ruaway
clear all
% close all
clc
load simulations/runaway/long43649.mat

% filtering
Ts = long_t(2)-long_t(1);
long_std_filter = lsim(c2d(tf(1,[0.004 1]),Ts),long_std,long_t);

% all - with iterations
figure()
plot(long_std,'--');

% restricted and filtered
start = 1.76*1e5;
stop = 1.81*1e5;
figure()
plot(long_t(start:stop),long_std(start:stop));
hold on
plot(long_t(start:stop),long_std_filter(start:stop));

% restricted and sampled
figure()
step = 15;
plot(long_t(start:stop),long_std_filter(start:stop),'--');
hold on
plot(long_t(start:step:stop),long_std_filter(start:step:stop),'o');