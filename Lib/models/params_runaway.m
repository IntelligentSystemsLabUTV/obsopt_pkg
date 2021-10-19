%% runaway init model
function params = params_runaway

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
    
    % initial condition
    params.X = [params.T0; params.W0];
    
    % position in the state vector of the parameters
    params.estimated_params = [];
end