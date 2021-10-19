%% params_pendulum
% create params structure for pendulum 
function params = params_pendulum_mass

    % pendulum parameters
    params.Lt = 1;
    params.M = 1;
    params.g = -9.81;
    params.Fw = 0.5;
    params.Fc = 0.5;
    params.If = 5;
    params.Iw = 3;
    params.Km = 1;
    
    % initial condition also with mass
    params.X(:,1) = [pi/2; 0.2; params.M];
    
    % position in the state vector of the parameters
    params.estimated_params = 3;
end