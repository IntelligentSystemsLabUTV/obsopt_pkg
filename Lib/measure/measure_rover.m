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
    y = zeros(params.OutDim,length(tspan));

    for k=1:length(tspan)
        t = tspan(k);
        % compute the time index
        pos = zeros(1,length(t));
        for i=1:length(t)
            tdiff = obs.setup.time-t(i);   
            pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
            pos(i) = max(1,pos(i));        
        end
    
        %%% get the IMU accelerations
        xd = obs.setup.model([t t+params.Ts],x(:,k),params,obs);
        IMU_true = xd(params.pos_v);
    
        %%% get distances        
        if mod(pos,params.UWB_samp) == 0            
            % true position
            p = x(params.pos_p);
            % adjacency matrix
            Pa(1,:) = x(params.pos_anchor(1):2:params.pos_anchor(end));
            Pa(2,:) = x(params.pos_anchor(2):2:params.pos_anchor(end));
            % true distances
            D = get_dist(p,Pa);       
        else
            D = 0*zeros(params.Nanchor,1);
        end
    
        % add noise
        % noise on UWB + IMU
        y(:,k) = [D; IMU_true];                 
    end
end