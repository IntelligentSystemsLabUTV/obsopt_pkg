%% params_pendulum
% create params structure for pendulum 
function params = params_double_integrator

    % control params
    params.K = -1;
    
    params.X(:,1) = [1;1;params.K];
    
    % position in the state vector of the parameters
    params.estimated_params = [];
    
    % which vars am i optimising
    params.opt_vars = [3];
    
    % not opt vars
    tmp = 1:length(params.X(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end