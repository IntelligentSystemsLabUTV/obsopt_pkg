%% MODEL_REFERENCE
% file: measure_general.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements the output mapping of a general
% state space system
% INPUT:
% x: state vector
% params: structure with all the necessary parameters 
% t: time instant (may be not used)
% OUTPUT:
% y: output measurement
function y = measure_rover(x,params,tspan,u,obs) 

    % define y
    y = zeros(params.OutDim,1);

    % get traj
    traj = obs.init.traj;
    
    t = tspan;
    % compute the time index
    pos = zeros(1,length(t));
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end

    %%% get the output mismatch terms           
    V_true = x(params.pos_v);  
    P_true = x(params.pos_p);

    %%% get distances        
    if mod(pos(end),params.UWB_samp) == 0                                    
        % adjacency matrix
        Pa(1,:) = x(params.pos_anchor(1):2:params.pos_anchor(end));
        Pa(2,:) = x(params.pos_anchor(2):2:params.pos_anchor(end));
        % true distances
        D = get_dist(P_true,Pa);   
        obs.init.params.last_D(traj,:) = D;
    else   
        D = reshape(obs.init.params.last_D(traj,:),params.Nanchor,1);
    end

    %%% get the IMU accelerations
    if mod(pos(end),params.IMU_samp) == 0                 
        xd = obs.setup.model([t t+params.Ts],x,params,obs);        
        IMU_true = xd(params.pos_v);          
        obs.init.params.last_IMU_acc(traj,:) = IMU_true;
    else       
        IMU_true = reshape(obs.init.params.last_IMU_acc(traj,:),params.space_dim,1);
    end

    % add noise
    % noise on UWB + IMU        
    y(:,1) = [D; P_true; V_true; IMU_true];                     
end