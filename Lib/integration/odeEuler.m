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
function out = odeEuler(ode,tspan,y0,options,varargin)

%%%% init section %%%%

% number of state's components
N = length(y0);            

% numer of time steps
M = length(tspan);         

% time step
dt = tspan(2) - tspan(1);   

% initial time instant
t0 = tspan(1);

% Matrices allocation
X = zeros(N,M);
X(:,1) = y0;
K1 = zeros(N,M);

% integration
for i = 1:M-1
    
    % State and time at ti
    x = X(:,i);
    
    % Runge Kutta 4
    [K1(:,i), Xjump] = feval(ode,t0,x);  

    %%% just to update the drive correctly %%%
    %tmp = feval(ode,t0,x);
    
    % Solution at ti+1
    X(:,i+1) = Xjump + dt*K1(:,i);

    % test quatnormalize
%     X(13:16,i+1) = quatnormalize(X(13:16,i+1)');
    
    % shift time instant
    t0 = tspan(i+1);
    
end

% any angles? - wrap them
% X(13,:) = wrapToPi(X(13,:));

% store
out.y = X;
end