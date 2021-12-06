%% 
clear all
close all
clc

%%
stepsize = 0.5;
x = -2:0.5:2;
y = x';
z = x.^2 + y.^2;
[px,py] = gradient(z,stepsize);

%% 
figure
contour(x,y,z)
hold on
quiver(x,y,px,py)
hold off