%% PLANT model and data
% model simulation
% plant data
function params = pendulum_init(varargin)

    Ts = varargin{1};
    t0 = varargin{2};
    tend = varargin{3};
    
    % get noise if exists
    if any(strcmp(varargin,'noise'))
        pos = find(strcmp(varargin,'noise'));
        noise = varargin{pos+1};
    else
        noise = 0;
    end
    
    if any(strcmp(varargin,'StateDim'))
        pos = find(strcmp(varargin,'StateDim'));
        StateDim = varargin{pos+1};
    else
        StateDim = 1;
    end
    
    if any(strcmp(varargin,'ObservedState'))
        pos = find(strcmp(varargin,'ObservedState'));
        ObservedState = varargin{pos+1};
    else
        ObservedState = 1;
    end
    
    % get model if exists
    if any(strcmp(varargin,'model'))
        pos = find(strcmp(varargin,'model'));
        model = varargin{pos+1};
    else
        model = 'test_model';
    end

    % get measure if exists
    if any(strcmp(varargin,'measure'))
        pos = find(strcmp(varargin,'measure'));
        measure = varargin{pos+1};
    else
        measure = 'test_measure';
    end
    
    % get noise if exists
    if any(strcmp(varargin,'ode'))
        pos = find(strcmp(varargin,'ode'));
        ode = varargin{pos+1};
    else
        ode = ode45;
    end
    
    
    % model simulation
    params.noise = noise;

    params.Lt = 1;
    params.M = 1;
    params.g = -9.81;
    params.Fw = 0.5;
    params.Fc = 0.5;
    params.If = 5;
    params.Iw = 3;
    params.Km = 1;
    
    params.u = 0;

    params.StateDim = StateDim;
    params.observed_state = ObservedState;
    params.OutDim = length(params.observed_state);
    
    params.X(:,1) = [pi/2;0];
    params.X_est(:,1) = params.X(:,1) + params.noise*([1e0;5e-1] + [5e-1;1e-1].*randn(params.StateDim,1));
    
    % setup simulation
    params.time = t0:Ts:tend;
    params.Niter = length(params.time);
    params.tspan = [0, Ts];

    % set model
    params.model = model;
    params.measure = measure;
    params.ode = ode;
    
end