clearvars
close all
clc

%% Plant
A = [-0.157, -0.094;
     -0.416, -0.45];
B = [0.87, 0.253, 0.743;
     0.39, 0.354, 0.65];
C = [0.0, 1.0];
D = [0.0, 0.0, 0.0];

sys_P = ss(A,B,C,D);

x0 = zeros(2, 1);

n = size(A, 1);
m = size(B, 2);
p = size(C, 1);

%% Controller
Ac = [-1.57, 0.5767, 0.822, -0.65;
     -0.9, -0.501, -0.94, 0.802;
     0.0, 1.0, -1.61, 1.614;
     0.0, 0.0, 0.0, 0.0];
Bc = [0.0;
      0.0;
      0.0;
      1.0];
Cc = [1.81, -1.2, -0.46, 0.0;
      -0.62, 1.47, 0.89, 0.0
      0.0, 0.0, 0.0, 0.0];
Dc = [0.0;
      0.0;
      0.0];

xc0 = zeros(4, 1);

%% Allocator with static annihilator
% Static annihilator
Pstar = dcgain(sys_P);
Pperp = null(Pstar);

q = size(Pperp, 2);

% Optimizer
gamma_all = 0.1;
R_all = eye(m);

Ag_all = zeros(q);
Bg_all = -gamma_all * Pperp' * R_all;

xg0_all = zeros(q, 1);

%% Allocator with dynamic annihilator
% Dynamic annihilator
[sys_An, ~, ~] = annihilator(sys_P);
Aan = sys_An.A;
Ban = sys_An.B;
Can = sys_An.C;
Dan = sys_An.D;

xan0_ann = zeros(size(Aan, 1), 1);

q = size(Ban, 2);

% Optimizer
gamma_ann = 1e8;
R_ann = eye(m);
Anstar = dcgain(sys_An);

Ag_ann = zeros(q);
Bg_ann = -gamma_ann * Anstar' * R_ann;
xg0_ann = zeros(q, 1);

%%
t0 = 0; dt = 1e-3; T = 50;
t = t0:dt:T;
r = ones(1, numel(t));

% Error
e = zeros(1, numel(t));
% Controller
xc = xc0;
% Allocator
J = zeros(1, numel(t));
% Input
u = zeros(m, numel(t));
% Plant
x = x0;
y = zeros(1, numel(t));
% Iterate on time
for it = 1 : numel(t)
  % Error
  e(it) = r(it) - y(max(it-1, 1));
  % Controller
  yc = Cc * xc + Dc * e(it);
  xc = xc + dt * (Ac * xc + Bc * e(it));
  % Input
  u(:, it) = yc;
  J(it) = 1/2 * u(:, it)' * R_all * u(:, it);
  % Plant
  y(it) = C * x + D * u(:, it);
  x = x + dt * (A * x + B * u(:, it));
end

% Error
e_all = zeros(1, numel(t));
% Controller
xc = xc0;
% Allocator
xg_all = xg0_all;
J_all = zeros(1, numel(t));
% Input
u_all = zeros(m, numel(t));
% Plant
x = x0;
y_all = zeros(1, numel(t));
% Iterate on time
for it = 1 : numel(t)
  % Error
  e_all(it) = r(it) - y_all(max(it-1, 1));
  % Controller
  yc = Cc * xc + Dc * e_all(it);
  xc = xc + dt * (Ac * xc + Bc * e_all(it));
  % Allocator
  ya_all = Pperp * xg_all;
  xg_all = xg_all + dt * (Ag_all * xg_all + Bg_all * (yc + ya_all));
  % Input
  u_all(:, it) = yc + ya_all;
  J_all(it) = 1/2 * u_all(:, it)' * R_all * u_all(:, it);
  % Plant
  y_all(it) = C * x + D * u_all(:, it);
  x = x + dt * (A * x + B * u_all(:, it));
end

% Error
e_ann = zeros(1, numel(t));
% Controller
xc = xc0;
% Allocator
xg_ann = xg0_ann;
J_ann = zeros(1, numel(t));
xa_ann = xan0_ann;
% Input
u_ann = zeros(m, numel(t));
% Plant
x = x0;
y_ann = zeros(1, numel(t));
% Iterate on time
for it = 1 : numel(t)
  % Error
  e_ann(it) = r(it) - y_ann(max(it-1, 1));
  % Controller
  yc = Cc * xc + Dc * e_ann(it);
  xc = xc + dt * (Ac * xc + Bc * e_ann(it));
  % Allocator
  ya_ann = Can * xa_ann;
  xg_ann = xg_ann + dt * (Ag_ann * xg_ann + Bg_ann * (yc + ya_ann));
  xa_ann = xa_ann + dt * (Aan * xa_ann + Ban * xg_ann);
  % Input
  u_ann(:, it) = yc + ya_ann;
  J_ann(it) = 1/2 * u_ann(:, it)' * R_ann * u_ann(:, it);
  % Plant
  y_ann(it) = C * x + D * u_ann(:, it);
  x = x + dt * (A * x + B * u_ann(:, it));
end

figure()
subplot(3, 2, 1)
hold on; grid on;
h1 = plot(t, r, '-k');
h2 = plot(t, y, '-r');
h3 = plot(t, y_all, '-g');
h4 = plot(t, y_ann, '-b');
xlim([t0 T])
legend([h1, h2, h3, h4], 'ref', 'no all', 'sta', 'dyn')
title('Output')
subplot(3, 2, 3)
hold on; grid on;
h1 = plot(t, y - y_all, '-g');
h2 = plot(t, y - y_ann, '-b');
xlim([t0 T])
legend([h1, h2], 'sta', 'dyn')
title('Output variation')
subplot(3, 2, 5)
hold on; grid on;
h1 = plot(t, J, '-r');
h2 = plot(t, J_all, '-g');
h3 = plot(t, J_ann, '-b');
xlim([t0 T])
legend([h1, h2, h3], 'no all', 'sta', 'dyn')
title('Cost function')
subplot(3, 2, 2)
hold on; grid on;
h1 = plot(t, u(1, :), '-r');
h2 = plot(t, u_all(1, :), '-g');
h3 = plot(t, u_ann(1, :), '-b');
xlim([t0 T])
legend([h1, h2, h3], 'no all', 'sta', 'dyn')
title('Input 1')
subplot(3, 2, 4)
hold on; grid on;
h1 = plot(t, u(2, :), '-r');
h2 = plot(t, u_all(2, :), '-g');
h3 = plot(t, u_ann(2, :), '-b');
xlim([t0 T])
legend([h1, h2, h3], 'no all', 'sta', 'dyn')
title('Input 2')
subplot(3, 2, 6)
hold on; grid on;
h1 = plot(t, u(3, :), '-r');
h2 = plot(t, u_all(3, :), '-g');
h3 = plot(t, u_ann(3, :), '-b');
xlim([t0 T])
legend([h1, h2, h3], 'no all', 'sta', 'dyn')
title('Input 3')
