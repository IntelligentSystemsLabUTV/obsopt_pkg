%% runaway init model
function params = params_runaway_gamma

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
    params.eps_coef = 10;

    % initial condition
    params.T0 = 9.255;
    params.W0 = 0.02176;
    
    % initial condition with gamma
    params.X = [params.T0; params.W0; params.gamma1; params.gamma];
    
    % position in the state vector of the parameters
    params.estimated_params = [3, 4];
end