%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function out = test_general(obs,N)

    %%%% Init Section %%%%
    x0_true = obs.init.X(1).val(:,1);
    obs.setup.Ts = 1e-2;
    obs.setup.t0 = 0;
    obs.setup.tend = 15;    
    obs.setup.time = obs.setup.t0:obs.setup.Ts:obs.setup.tend;
    obs.setup.Niter = length(obs.setup.time);
    [filter, filterScale] = filter_define(obs.setup.Ts,obs.setup.Nts);
    obs.setup.filterTF = filter;
    obs.setup.Nfilt = length(filterScale)-1;

    x0_dist = zeros(obs.setup.dim_state,N);
    for i=1:N
        x0_dist(:,i) = x0_true;
        x0_dist(obs.setup.plot_vars,i) =  x0_true(obs.setup.plot_vars) + 0*1e-1*randn(length(obs.setup.plot_vars),1);
    end
    
    out.time = obs.setup.time;

    % true system - correct initial condition and no noise
    % considered
    X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, obs.init.params, obs), obs.setup.time, x0_true ,obs.init.params.odeset); 
    obs.init.X(obs.init.traj).val = X.y;
    out.x_true.val = X.y;
        
    % consider this trajectory
    obs.init.traj = 1;
    
    % reset filter state
    if obs.setup.Nfilt
        obs.init.X_filter(1).val{:,:} = zeros(1,obs.setup.Niter);
    end
    % reset measurements
    obs.init.Y_full_story(1).val = zeros(obs.setup.Nfilt+1,obs.setup.dim_out,0);
    
    % set measurements
    % here the noise is noise added aggording to noise_spec
    for i=1:obs.setup.Niter
        obs.init.Ytrue_full_story(1).val(1,:,i) = obs.setup.measure_reference(out.x_true.val(:,i),obs.init.params,obs.setup.time(i));
        obs.init.noise_story(1).val(:,i) = obs.setup.noise*(obs.setup.noise_mu(obs.setup.observed_state)  + obs.setup.noise_std(obs.setup.observed_state).*randn(obs.setup.dim_out,1));        
        y(1).val =  reshape(obs.init.Ytrue_full_story(1).val(1,:,i),obs.setup.dim_out,1) + obs.init.noise_story(1).val(:,i);
        obs.init.Y_full_story(1).val(1,:,i) = y(1).val;
        
        if obs.setup.Nfilt
            tspan_pos = [max(1,i-1), i];
            [dy, x_filter] = obs.measure_filter(obs.init.Y_full_story(1).val,tspan_pos,obs.init.X_filter(1));                
            for filt=1:obs.setup.Nfilt
                for dim=1:obs.setup.dim_out
                    y_tmp(dim,1) = dy{filt,dim}.val(end);
                    obs.init.X_filter(1).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                end   
                y(1).val = [y(1).val, y_tmp];
            end
        end
        
        % insert measurements
        for term=1:obs.setup.J_nterm                        
            obs.init.Y_full_story(1).val(term,:,i) = y(1).val(:,term);             
        end
    end

    % integration loop
    for i=1:N
        
        % Display iteration step  
        clc
        disp(['Iteration Number: ', num2str(i),'/',num2str(N)])                                                        

        % real system - initial condition perturbed 
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), obs.setup.time, x0_dist(:,i),obs.init.params.odeset);
        out.x_dist(i).val = X.y;      

    end
    
    %%% measure %%%
    out.y_true.val = obs.setup.measure_reference(out.x_true.val, obs.init.params, obs.setup.time);
    
    for i=1:N
        out.y_dist(i).val = obs.setup.measure(out.x_dist(i).val, obs.init.params, obs.setup.time);
    end

end



