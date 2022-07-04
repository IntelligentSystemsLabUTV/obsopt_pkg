%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE obs_outerver on general model
% INPUT: none
% OUTPUT: params,obs_out
function [out, obs_out] = test_general_obs(obs,N)

    obs_out = obs;

    %%%% Init Section %%%%
    x0_true = obs_out.init.X(1).val(:,1);
    obs_out.setup.Ts = 1e-2;
    obs_out.setup.t0 = 0;
    obs_out.setup.tend = 15;    
    obs_out.setup.time = obs_out.setup.t0:obs_out.setup.Ts:obs_out.setup.tend;
    obs_out.setup.Niter = length(obs_out.setup.time);
    [filter, filterScale] = filter_define(obs_out.setup.Ts,obs_out.setup.Nts);
    obs_out.setup.filterTF = filter;
    obs_out.setup.Nfilt = length(filterScale)-1;

    x0_dist = zeros(obs_out.setup.dim_state,N);
    for i=1:N
        x0_dist(:,i) = x0_true;
        x0_dist(obs_out.setup.plot_vars,i) =  0*x0_true(obs_out.setup.plot_vars) + 1*5e-1*randn(length(obs_out.setup.plot_vars),1);
    end
    
    out.time = obs_out.setup.time;

    % true system - correct initial condition and no noise
    % considered
    X = obs_out.setup.ode(@(t,x)obs_out.setup.model_reference(t, x, obs_out.init.params, obs_out), obs_out.setup.time, x0_true ,obs_out.init.params.odeset); 
    obs_out.init.X(obs_out.init.traj).val = X.y;
    out.x_true.val = X.y;
        
    % consider this trajectory
    obs_out.init.traj = 1;
    
    % reset filter state
    if obs_out.setup.Nfilt
        obs_out.init.X_filter(1).val{:,:} = zeros(1,obs_out.setup.Niter);
    end
    % reset measurements
    obs_out.init.Y_full_story(1).val = zeros(obs_out.setup.Nfilt+1,obs_out.setup.dim_out,0);
    
    % set measurements
    % here the noise is noise added aggording to noise_spec
    for i=1:obs_out.setup.Niter
        obs_out.init.Ytrue_full_story(1).val(1,:,i) = obs_out.setup.measure_reference(out.x_true.val(:,i),obs_out.init.params,obs_out.setup.time(i));
        obs_out.init.noise_story(1).val(:,i) = obs_out.setup.noise*(obs_out.setup.noise_mu(obs_out.setup.observed_state)  + obs_out.setup.noise_std(obs_out.setup.observed_state).*randn(obs_out.setup.dim_out,1));        
        y(1).val =  reshape(obs_out.init.Ytrue_full_story(1).val(1,:,i),obs_out.setup.dim_out,1) + obs_out.init.noise_story(1).val(:,i);
        obs_out.init.Y_full_story(1).val(1,:,i) = y(1).val;
        
        if obs_out.setup.Nfilt
            tspan_pos = [max(1,i-1), i];
            [dy, x_filter] = obs_out.measure_filter(obs_out.init.Y_full_story(1).val,tspan_pos,obs_out.init.X_filter(1));                
            for filt=1:obs_out.setup.Nfilt
                for dim=1:obs_out.setup.dim_out
                    y_tmp(dim,1) = dy{filt,dim}.val(end);
                    obs_out.init.X_filter(1).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                end   
                y(1).val = [y(1).val, y_tmp];
            end
        end
        
        % insert measurements
        for term=1:obs_out.setup.Nfilt+1                        
            obs_out.init.Y_full_story(1).val(term,:,i) = y(1).val(:,term);             
        end
    end
    
    % get measures
    out.y_true.val = obs_out.init.Y_full_story(1).val;

    % integration loop
    for i=1:N
        
        % Display iteration step  
        clc
        disp(['Iteration Number: ', num2str(i),'/',num2str(N)])                                                        

        % real system - initial condition perturbed 
        X = obs_out.setup.ode(@(t,x)obs_out.setup.model(t, x, obs_out.init.params, obs_out), obs_out.setup.time, x0_dist(:,i),obs_out.init.params.odeset);
        out.x_dist(i).val = X.y;      

    end        
    
    for i=1:N        
        out.x_filter(i).val{:,:} = zeros(1,obs_out.setup.Niter);
        
        for iter=1:obs_out.setup.Niter 
            
            out.y_dist(i).val(1,:,iter) = obs_out.setup.measure(out.x_dist(i).val(:,iter), obs_out.init.params, obs_out.setup.time(iter));
        
            if obs_out.setup.Nfilt
                tspan_pos = [max(1,iter-1), iter];
                [dy, x_filter] = obs_out.measure_filter(out.y_dist(i).val,tspan_pos,out.x_filter(i));                
                for filt=1:obs_out.setup.Nfilt
                    for dim=1:obs_out.setup.dim_out
                        y_tmp(dim,1) = dy{filt,dim}.val(end);
                        out.x_filter(i).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                    end   
                    y(1).val = [y(1).val, y_tmp];
                end
            end
        
            % insert measurements
            for term=1:obs_out.setup.Nfilt+1
                out.y_dist(i).val(term,:,i) = y(1).val(:,term);             
            end
        end
    end

end



