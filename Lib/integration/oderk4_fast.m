%% runge kutta 4 integration
function out = oderk4_fast(ode,tspan,y0,options,varargin)

N = length(y0);             % number of state's components
M = length(tspan);         % numer of time steps
dt = tspan(2) - tspan(1);   % time step
t0 = tspan(1);

% Matrices allocation
X = zeros(N,M);
X(:,1) = y0;
K1 = zeros(N,M);

for i = 1:M-1
    
    % State and time at ti
    x = X(:,i);
    
    % Runge Kutta 4
    K1(:,i) = feval(ode,t0,y0);
    K2 = feval(ode,t0,x + K1(:,i)*dt/2);
    K3 = feval(ode,t0,x + K2*dt/2);
    K4 = feval(ode,t0,x + K3*dt);
    
    % Solution at ti+1
    X(:,i+1) = x + (dt/6)*(K1(:,i) + 2*K2 + 2*K3 + K4);
    
end

% store
out.y = X;


end