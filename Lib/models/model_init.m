%% PLANT model and data
% model simulation
% plant data
% this function defines the model parameters. It's a general method,
% depending only on the parameters_init.m script, defining the model
% variables and costants. 
function params = model_init(varargin)

    % get params_init.m script
    if any(strcmp(varargin,'params_init'))
        pos = find(strcmp(varargin,'params_init'));
        params_init = varargin{pos+1};
        params = params_init();
    else
        params.X = 5;
    end
    
    % handle parameters setup (identification task)
    fields = fieldnames(params);
    if ~any(strcmp(fields,'estimated_params'))
        params.estimated_params = [];
    end
    
    % sample time
    if any(strcmp(varargin,'Ts'))
        pos = find(strcmp(varargin,'Ts'));
        Ts = varargin{pos+1};
    else
        Ts = 1e-1;
    end
    
    % starting and finish time
    if any(strcmp(varargin,'T0'))
        pos = find(strcmp(varargin,'T0'));
        time_tmp = varargin{pos+1};
        t0 = time_tmp(1);
        tend = time_tmp(2);
    else
        t0 = 0;
        tend = 1;
    end

    % setup the structure
    params.time = t0:Ts:tend;
    params.Ts = Ts;
    params.Niter = length(params.time);
    params.tspan = [0, Ts];
    
    % get noise if exists. Default is 0
    if any(strcmp(varargin,'noise'))
        pos = find(strcmp(varargin,'noise'));
        params.noise = varargin{pos+1};
    else
        params.noise = 0;
    end
    
    % get noise setup if exists. Default is mu=0, std=5e-2;
    if any(strcmp(varargin,'noise_spec'))
        pos = find(strcmp(varargin,'noise_spec'));
        noise_spec = varargin{pos+1};
        params.noise_mu = noise_spec(1);
        params.noise_std = noise_spec(2);
    else
        params.noise_mu = 0;
        params.noise_std = 5e-2;
    end
    
    % get state dimension. Default is 1
    if any(strcmp(varargin,'StateDim'))
        pos = find(strcmp(varargin,'StateDim'));
        params.StateDim = varargin{pos+1};
    else
        params.StateDim = 1;
    end
    
    % get set of observed states. Default is 1
    if any(strcmp(varargin,'ObservedState'))
        pos = find(strcmp(varargin,'ObservedState'));
        params.observed_state = varargin{pos+1};
    else
        params.observed_state = 1;
    end
    % set the output dimensions from the observed state
    params.OutDim = length(params.observed_state);
    
    % get model if exists. Default is a 1 dimension asymptotically stable
    % system.
    if any(strcmp(varargin,'model'))
        pos = find(strcmp(varargin,'model'));
        params.model = varargin{pos+1};
    else
        params.model = @(t,x,params) -2*x;
    end

    % get measure if exists. Default measures the whole state
    if any(strcmp(varargin,'measure'))
        pos = find(strcmp(varargin,'measure'));
        params.measure = varargin{pos+1};
    else
        params.measure = @(x,params) x;
    end
    
    % get the integration algorithm. Default is ode45
    if any(strcmp(varargin,'ode'))
        pos = find(strcmp(varargin,'ode'));
        params.ode = varargin{pos+1};
    else
        params.ode = @ode45;
    end

    % input 
    if any(strcmp(varargin,'input'))
        pos = find(strcmp(varargin,'input'));
        params.input = varargin{pos+1};
    else
        params.input = 0*sin(params.time);
    end
    % set initial input
    params.u = params.input(:,1);
    
    % set initial condition perturbed
    perc = 0.1;
%     params.X_est(:,1) = params.X(:,1).*(1 + perc*params.noise*randn(params.StateDim,1));
    params.X_est(:,1) = params.X(:,1).*(1 + perc*params.noise*ones(params.StateDim,1));
    
end