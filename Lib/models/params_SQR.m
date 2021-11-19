%% runaway init model
function params = params_SQR

    % plant data

    % electric charge
    params.a = 0.9;
    params.b = 0.4;

    % eps_coef
    params.eps_coef = 1;
    
    % initial condition
    params.X = [2; 0; 1; params.a; params.b];
    
    % position in the state vector of the parameters
    params.estimated_params = [4, 5];
end