%% runge kutta 4 integration
function out = odeDD(varargin)

ode = varargin{1};
tspan = varargin{2};
x0 = varargin{3};
U = varargin{4};
TF = varargin{5};

N = length(x0);           
  % number of state's components
M = length(tspan);         % numer of time steps
t0 = tspan(1);

% Matrices allocation
X = zeros(N,M);
X(:,1) = x0;
Y(:,1) = 0*U(:,1);

for i = 1:M-1
    
    % DD step
    [Y(:,i+1),X(:,i+1)] = feval(ode,tspan(i),X(:,i),U,TF.A, TF.B, TF.C, TF.D);   
    
end

% store
out.x = X;
out.y = Y(:,2:end);


end