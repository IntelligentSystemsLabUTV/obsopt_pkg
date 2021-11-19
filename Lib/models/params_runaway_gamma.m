%% runaway init model
function params = params_runaway_gamma

    % plant data

    % electric charge
    params.Q = 1;

    % spontaneous emission
    params.S = 0;

    % parameters
    params.gamma = 0.5;
    params.gamma1 = 2.5;
    params.ni = 0.5;
    params.Wt = 0.1;

    % eps_coef
    params.eps_coef = 1;

    % initial condition
    params.T0 = 0;
    params.W0 = 0.001;

    % initial condition
    params.T0 = 0;
    params.W0 = 0.001;
    
    % initial condition with gamma
    params.X = [params.T0; params.W0; params.gamma1; params.gamma];
    
    % position in the state vector of the parameters
    params.estimated_params = [3, 4];
end