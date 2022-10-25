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
    x_dot = repmat(x,1,length(tspan)-1);    
    t = tspan(1);
    
    % compute the time index
    tdiff = obs.setup.time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
    pos = max(1,min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3)));
    drive_out = [];
    params.u = params.input(tspan,drive_out,params);   
    
    % compute y out of p
    y = obs.setup.measure_reference(x,params,t,obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos-1)));
    
    % compute the reference (Sigma_r)
    range = 1:params.dim_state_r;
    x_dot(range,:) = params.Ar*x(range,:) + params.Br*0; 
    r = params.Cr*x(range,:) + params.Dr*params.u;            
    
    % compute the CL plant
    range = params.dim_state_r+1:params.dim_state - params.NumPsi - params.NumGamma;    
    r_lsim = r;
    x0 = x(range);    
    [~,~,xout] = lsim(params.sys_pert(obs.init.traj).sys_CL_All,r_lsim,tspan,x0);  
    x_dot(range,:) = xout(2:end,:)';
    
     % save input story
     if ~params.optimising
        [y,~,~] = lsim(params.sys_pert(obs.init.traj).sys_CL_Allu,r_lsim,tspan,x0);  
        obs.init.input_story(obs.init.traj).val(:,pos) = y(2:end,:)';  
     end
       
end