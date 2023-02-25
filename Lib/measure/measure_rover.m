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

    %%% when do I stop? %%%
    t = tspan;
    if numel(t)>2
        stopK = numel(t);
        offset_UWBsamp = 0;
    else
        stopK = max(1,numel(t)-1);
        offset_UWBsamp = 1;
    end

    % define y
    y = zeros(params.OutDim,stopK);

    % get traj
    traj = obs.init.traj;       

    % compute the time index
    pos = zeros(1,length(t));
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end
    
    for k=1:stopK
        
    
        %%% get the output mismatch terms        
        V_true = reshape(x(params.pos_v,k),numel(params.pos_v),1);
        P_true = reshape(x(params.pos_p,k),numel(params.pos_p),1);
    
        %%% get distances        
        if mod(pos(k)+offset_UWBsamp,params.UWB_samp) == 0                                    
            % adjacency matrix
            for dim=1:params.space_dim
                Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
            end
            % true distances
            D = get_dist(P_true,Pa);   
            obs.init.params.last_D(traj,:) = D;
        else   
            D = reshape(obs.init.params.last_D(traj,:),params.Nanchor,1);
        end
    
        %%% get the IMU accelerations
        if mod(pos(k)+offset_UWBsamp,params.IMU_samp) == 0                 
            xd = obs.setup.model([t(k) t(k)+params.Ts],x,params,obs);        
            IMU_true = reshape(xd(params.pos_v,:),numel(params.pos_v),1);          
            obs.init.params.last_IMU_acc(traj,:) = IMU_true;
        else       
            IMU_true = reshape(obs.init.params.last_IMU_acc(traj,:),params.space_dim,1);
        end
    
        % add bias on IMU
        IMU_true = IMU_true;

        % final measure
        y(:,k) = [D; P_true; V_true; IMU_true];                     
    end
end