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
function y = measure_rover_bearing(x,params,tspan,u,obs) 

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
        
    
        %%% get the output mismatch terms - position
        P_true = reshape(x(params.pos_p,k),numel(params.pos_p),1);
        
        %%% get the output mismatch terms - velocity
        [obs.init.params.xdhat(traj).val(:,end+1), obs.init.params.xdhatbuf, obs.init.params.xdhatcount(traj).val] = PseudoDer( ...
        params.Ts,P_true,params.cder,params.dder,3,0,0,obs,obs.init.params.xdhatbuf,obs.init.params.xdhatcount(traj).val);  
        V_true = obs.init.params.xdhat(traj).val(:,end);
        
        % get bearing - possibly mismatch term, no actual measurement
        Theta_true = x(params.pos_theta);
    
        %%% get distances        
        if mod(pos(k)+offset_UWBsamp,params.UWB_samp) == 0                                    
            % adjacency matrix
            for dim=1:params.space_dim
                Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
            end
            % true distances
            theta = x(params.pos_theta);
            delta = params.dist_tag*[sin(theta); -cos(theta); 0];
            DT1 = get_dist(P_true+delta,Pa(:,params.TagAnchorIndex(1,:)));
            DT2 = get_dist(P_true-delta,Pa(:,params.TagAnchorIndex(2,:)));
            D = [DT1; DT2];   
            obs.init.params.last_D(traj,:) = D;
        else   
            D = reshape(obs.init.params.last_D(traj,:),params.Nanchor,1);
        end
    
        % get the IMU measurements        
        [obs.init.params.xddhat(traj).val(:,end+1), obs.init.params.xddhatbuf, obs.init.params.xddhatcount(traj).val] = PseudoDer( ...
        params.Ts,V_true,params.cder,params.dder,3,0,0,obs,obs.init.params.xddhatbuf,obs.init.params.xddhatcount(traj).val);    
        IMU_true = obs.init.params.xddhat(traj).val(:,end);

        % bias
        if params.EKF || params.sferbias
            IMU_true = IMU_true + params.bias*(x(params.pos_bias));
        end                

        % final measure
        y(:,k) = [D; P_true; V_true; IMU_true; Theta_true];                     
    end
end