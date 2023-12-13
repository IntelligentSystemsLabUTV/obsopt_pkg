%% ODEDD
% file: odeDD.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements a discrete time system integration
% algorithm
% INPUT:
% varargin: general parameters
% OUTPUT:
% out: structure with integration results
function out = odeTF(varargin)

% init params
nargin = length(varargin);

% handle function to the dynamics
ode = varargin{1};

% tspan: time interval for the integration
tspan = varargin{2};

% y0: initial condition
x0 = varargin{3};

% U: input
U = varargin{4};

% TF: transfer function
TF = varargin{5};

% number of state's components
N = length(x0);           
  
% numer of time steps
M = length(tspan);   

% initial time instant
t0 = tspan(1);

% Matrices allocation
X = zeros(N,M);
X(:,1) = x0;
Y(:,1) = 0*U(:,1);

% integration
for i = 1:M-1
    
    % DD step
    [Y(:,i+1),X(:,i+1)] = feval(ode,tspan(i),X(:,i),U,TF.A, TF.B, TF.C, TF.D);   
    
end

% store
out.x = X;
out.y = Y(:,2:end);
end