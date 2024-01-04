%% odeEuler
% file: oderk4_fast.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function implements a forward Euler integration
%              algorithm
% INPUT:
%           ode: handle function to the dynamics
%           tspan: time interval for the integration
%           y0: initial condition
%           options: integration options (see ode45 help could be used)
%           varargin: additional parameters (see options)
% OUTPUT:
%           out: structure with integration results
function out = odeEuler(ode,tspan,y0,options,varargin)

    % init section

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
    K1 = zeros(N,M);

    % set initial condition
    X(:,1) = y0;

    % integration: cycle over the number of time steps. We integrate M-1 times
    % because every time we integrate into the next step.
    for i = 1:M-1
    
        % State and time at t_i
        x = X(:,i);
    
        % evaluate the model
        % remark: check the structure of the model to understand what is Xjump
        [K1(:,i), Xjump] = feval(ode,t0,x);  
    
        % Solution at ti+1 (forward Euler)
        X(:,i+1) = Xjump + dt*K1(:,i);
    
        % shift time instant
        t0 = tspan(i+1);
    
    end

    % store
    out.y = X;

end