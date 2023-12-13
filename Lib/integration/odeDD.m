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
function out = odeDD(varargin)

% init params
nargin = length(varargin);

% handle function to the dynamics
ode = varargin{1};

% tspan: time interval for the integration
tspan = varargin{2};

% y0: initial condition
x0 = varargin{3};

% integrations options
if nargin > 3
    opts = varargin{4};
end

% number of state's components
N = length(x0);           
  
% numer of time steps
M = length(tspan);

% initial time instant
t0 = tspan(1);

% Matrices allocation
X = zeros(N,M);
X(:,1) = x0;

% integration
for i = 1:M-1
    
    % DD step
    X(:,i+1) = feval(ode,tspan(i),X(:,i));   
    
end

% store
out.y = X;
end