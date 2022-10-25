%% ODERK4_FAST
% file: oderk4_fast.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements a 4th order Runge-Kutta integration
% algorithm
% INPUT:
% ode: handle function to the dynamics
% tspan: time interval for the integration
% y0: initial condition
% options: integration options (see ode45 help)
% varargin: additional parameters (see options)
% OUTPUT:
% out: structure with integration results
function out = odeLsim(ode,tspan,y0,options,varargin)

%%%% init section %%%%
out.y(:,1) = y0;
Nstep = length(tspan);

% store
out.y(:,2:Nstep) = feval(ode,tspan,y0);
end