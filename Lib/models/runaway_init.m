%% PLANT model and data
function params = runaway_init(varargin)

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
    
    % plant data

    % electric charge
    params.Q = 1;

    % spontaneous emission
    params.S = 0.01;

    % parameters
    params.gamma = 0.5;
    params.gamma1 = 4;
    params.ni = 0.5;
    params.Wt = 0.1;

    % ringing
    params.wq = 1;
    params.chi = 1;

    % eps_coef
    params.eps_coef = 1;

    % initial condition
    params.T0 = 9.255;
    params.W0 = 0.02176;
%     params.T0 = 0;
%     params.W0 = 0.001;
    
    params.X(:,1) = [params.T0; params.W0];
    params.X_est(:,1) = [params.T0; params.W0] + params.noise*[2; 5e-1].*randn(2,1);

    params.StateDim = StateDim;
    params.observed_state = ObservedState;
    params.OutDim = length(params.observed_state);
    
    % setup simulation
    params.time = t0:Ts:tend;
    params.Niter = length(params.time);
    params.tspan = [0, Ts];

    % set model
    params.model = model;
    params.measure = measure;
    params.ode = ode;
end