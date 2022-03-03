%% params_pendulum
% create params structure for pendulum 
function params = params_Tesi_01
    
    % control
    for i=1:3
       for j=1:3
          eval(['params.KX',num2str(i),num2str(j),' = 0;']); 
          eval(['params.KR',num2str(i),num2str(j),' = 0;']);
       end
    end
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    
    % reference init
    params.dim_state = 3;
    str_tmp = 'params.X(1).val(:,1) = [[1;1;1]*1e0;';
    for i=1:3
       for j=1:3
          str_tmp = [str_tmp, 'params.KX',num2str(i),num2str(j),';'];         
       end
    end
    for i=1:3
       for j=1:3
          str_tmp = [str_tmp, 'params.KR',num2str(i),num2str(j),';'];         
       end
    end
    str_tmp = [str_tmp(1:end-1),'];'];
    eval(str_tmp);
    
    % assign to other traj
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    dim_state_aug = length(params.X(1).val(:,1));
    params.estimated_params = [params.dim_state+1:dim_state_aug];
    
    % which vars am I optimising
    params.opt_vars = [params.dim_state+1:dim_state_aug];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars
    params.plot_vars = 3;
end