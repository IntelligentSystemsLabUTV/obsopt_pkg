%% params_pendulum
% create params structure for pendulum 
function params = params_mockup

    % pendulum parameters
    params.theta = 0.5;
    
    
    params.X(:,1) = [1;params.theta];
    
    % position in the state vector of the parameters
    params.estimated_params = [2];
end