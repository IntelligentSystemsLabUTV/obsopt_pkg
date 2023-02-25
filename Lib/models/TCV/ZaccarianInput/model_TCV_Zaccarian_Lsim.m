%% MODEL_OSCILLATOR_VDP
% file: model_control_test.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the dynamics equation of an unstable
% LTI model
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_TCV_Zaccarian_Lsim(tspan,x,params,obs)

    % init the dynamics         
    x_dot = repmat(x,1,max(1,max(1,length(tspan)-1)));    
    t = tspan(1);
    
    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end
    drive_out = [];
    params.u = params.input(tspan,drive_out,params);   
    
    
    % compute the reference (Sigma_r)
    range = 1:params.dim_state_r;
    x_dot(range,:) = params.Ar*x(range,:) + params.Br*0; 
    r = params.Cr*x(range,:) + params.Dr*params.u;   
    
    % shift tspan
    tspan = tspan-tspan(1);
    
    % compute the CL plant
    range = params.dim_state_r+1:params.dim_state - params.NumPsi - params.NumGamma;    
    r_lsim = r;
    x0 = x(range);        
    [y,~,xout] = lsim(params.sys_pert(obs.init.traj).sys_CL_All,r_lsim,tspan,x0,'foh');
    x_dot(range,:) = xout(2:end,:)';
    
     % save input story     
    [u,~,~] = lsim(params.sys_pert(obs.init.traj).sys_CL_Allu,r_lsim,tspan,x0,'foh');  
    obs.init.input_story(obs.init.traj).val(:,pos) = u';  
    obs.init.input_pos_vec = pos;
    
    % save error
    rbar = r(:,params.q_pos);
    ybar = y(:,params.q_pos);    
    e = rbar-ybar;
    obs.init.error_story_ref(obs.init.traj).val(:,pos) = e';
       
end