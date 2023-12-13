%% Examples of |odehybrid|
%
% This script contains numerous examples of using |odehybrid| to simulate
% continuous and discrete systems, from a simple example to more complex 
% examples with non-vector states and logging, introducing all primary 
% features of |odehybrid|. It's meant for viewing directly in MATLAB by 
% clicking "Publish" in the editor or entering:
%
%   home = fileparts(which('examples_odehybrid'));
%   web(fullfile(home, 'html', 'examples_odehybrid.html'));
% 
% at the command line. This will run the code and open the result as HTML.
% 
% For additional discussion, see:
% 
% * Online documentation (<http://www.anuncommonlab.com/doc/odehybrid/>)
% * Discussion of how simulations work (<http://www.anuncommonlab.com/articles/how-simulations-work/>)
% * Motivations behind the tool (<http://www.anuncommonlab.com/blog/a-simulation-engine/>)
%
% Copyright 2014 An Uncommon Lab

%%
% <html><center><h1>Basics</h1></center></html>

%% Short and Direct
% Let's say we have a continuous system in the form:
%
% $$\dot{x}_c(t) = f_c(t, x_c(t), x_d(t))$$
%
% where $x_c$ is the continuous state and $x_d$ is some state updated at a 
% discrete time step as
%
% $$x_d(t+\Delta t) = f_d(t, x_c(t), x_d(t))$$
% 
% Here, we'll give a quick embodiment of this system to show how we can
% simulate it from its initial condition over time. Afterwards, we'll go
% more into how it works.
% 
% Let |ode|, |de|, |dt|, |x|, and |u| be $f_c$, $f_d$, $\Delta t$, $x_c$, 
% and $x_d$, respectively.

% define params
params.decay = 0.01;
params.theta = 0.5;

dt  = 0.1;                                     % Discrete eq. time step
ts  = [0 5];                                   % From 0 to 5s
xf0  = 1;                                      % Initial continuous state
xj0  = 0;                                      % Initial discrete state
[tf, xf, tj, xj] = odehybrid(@ode45, @(x,t,parans)flow_map(x,t,params), @(x,t,params)jump_map, dt, ts, xf0, xj0,params); % Simulate!
plot(tf, xf, tj, xj, '.'); xlabel('Time');                     % Plot 'em.
legend('x_f', 'x_j', 'Location', 'se');                % Label 'em.

%%% the functions will be a little more involved
function x_dot = flow_map(x,t,params)
    x_dot(1) = -params.decay*x(1);
end

function x_plus = jump_map(x,t,params)
    x_plus(1) = params.theta*x(1) + (1-params.theta)*rand;
end



