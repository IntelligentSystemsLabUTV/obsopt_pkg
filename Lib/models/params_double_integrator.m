%% params_pendulum
% create params structure for pendulum 
function params = params_double_integrator

    % control params
    params.K1 = -3;
    params.K2 = -2;
    params.K3 = -1;
    
    params.X(1).val(:,1) = [1;1;params.K1; params.K2; params.K3];
    
%     params.X(2).val(:,1) = [2;2;params.K1; params.K2];
%     params.X(2).val(:,1) = params.X(1).val(:,1);
    
    % position in the state vector of the parameters
    params.estimated_params = [3 4 5];
    
    % which vars am i optimising
    params.opt_vars = [3 4 5];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end