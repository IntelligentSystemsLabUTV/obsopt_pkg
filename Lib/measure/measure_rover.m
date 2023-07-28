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
        Quat_true = reshape(x(params.pos_quat,k),numel(params.pos_quat),1);

        % place the tags
        R = quat2rotm(Quat_true.');
        for i=1:3
            Pt(:,i) = R*params.TagPos(:,i) + P_true;
        end
        Pt(3,:) = Pt(3,:) - params.TagPos(3,:);
    
        % different sampling times   
        if mod(pos(k)+offset_UWBsamp,params.UWB_samp) == 0 

            %%% get distances 
            % adjacency matrix
            for dim=1:params.space_dim
                Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
            end

            % true distances
            D = get_dist(Pt,Pa);   
            obs.init.params.last_D(traj,:) = D;

            % orientation          
            obs.init.params.last_Quat(traj,:) = Quat_true;

        else   
            D = reshape(obs.init.params.last_D(traj,:),3*params.Nanchor,1);
            Quat_true = reshape(obs.init.params.last_Quat(traj,:),4,1);
        end
    
        %%% get the IMU accelerations              
        IMU_true = reshape(x(params.pos_acc,k),numel(params.pos_acc),1); 

        %%% get the Gyro velocities              
        W_true = reshape(x(params.pos_w,k),numel(params.pos_w),1); 

        % bias
        if params.sferbias
            IMU_true = IMU_true + params.bias*(x(params.pos_bias));
        end

        % final measure
        y(:,k) = [D; P_true; V_true; IMU_true; Quat_true; W_true];                     
    end
end