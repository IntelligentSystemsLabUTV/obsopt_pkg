%% params_pendulum
% create params structure for pendulum 
function params = params_mockup_chaos
    
    params.X(:,1) = [5;5];
    
    % position in the state vector of the parameters
    params.estimated_params = [];
end