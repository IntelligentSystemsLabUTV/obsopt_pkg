%% runaway init model
function params = params_Rossler

    % plant data

    % electric charge
    params.a = 0.5;
    params.b = 1;
    params.c = 3;

    % eps_coef
    params.eps_coef = 1;
    
    % initial condition
    params.X = [2; 0; 1; params.a; params.b; params.c];
    
    % position in the state vector of the parameters
    params.estimated_params = [4, 5, 6];
end