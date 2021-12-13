%% params_pendulum
% create params structure for pendulum 
function params = params_double_integrator

    % control params
    params.K1 = -2;
    params.K2 = -3;
    
    params.X(:,1) = [1;1;params.K1; params.K2];
    
    % position in the state vector of the parameters
    params.estimated_params = [3, 4];
    
    % which vars am i optimising
    params.opt_vars = [3, 4];
    
    % not opt vars
    tmp = 1:length(params.X(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end