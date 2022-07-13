%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE obs_outerver on general model
% INPUT: none
% OUTPUT: params,obs_out
function out = test_general_obs(obs,N)

    %%%% Init Section %%%%
    x0_true = 0*obs.init.X(1).val(:,1);
    Ts = 1e-2;
    t0 = 0;
    tend = 10;    
    time = t0:Ts:tend;
    Niter = length(time);
    
    [filter, filterScale] = filter_define(Ts,1);
    obs_tmp = obsopt( 'filters', filterScale,'filterTF', filter,'params',obs.init.params);                    
    obs_tmp.setup.Nfilt = length(filterScale)-1;
    Nfilt = obs_tmp.setup.Nfilt;
    obs_tmp.setup.dim_state = obs.setup.dim_state;
    obs_tmp.setup.plot_vars = obs.setup.plot_vars; 
    obs_tmp.setup.ode = obs.setup.ode;
    obs_tmp.setup.model_reference = obs.setup.model_reference;
    obs_tmp.setup.model = obs.setup.model;
    obs_tmp.setup.measure_reference = obs.setup.measure_reference;
    obs_tmp.setup.measure = obs.setup.measure;
    obs_tmp.setup.params = obs.init.params;
    obs_tmp.setup.params.input_enable = 1;
    obs_tmp.init.params = obs_tmp.setup.params;
    obs_tmp.setup.time = time;
    obs_tmp.setup.Ts = Ts;
    obs_tmp.init.params.odeset = obs.init.params.odeset;      
    obs_tmp.setup.dim_out = obs.setup.dim_out;
    obs_tmp.setup.observed_state = obs.setup.observed_state;
    obs_tmp.init.params.odeset = obs.init.params.odeset;
    
    obs_tmp.setup.noise = 0;
    noise_mu = zeros(obs_tmp.setup.dim_state,1);
    noise_std = zeros(obs_tmp.setup.dim_state,1);
    obs_tmp.init.traj = 1;  

    x0_dist = zeros(obs_tmp.setup.dim_state,N);
    for i=1:N
        x0_dist(:,i) = x0_true;
        x0_dist(obs_tmp.setup.plot_vars,i) =  0*x0_true(obs_tmp.setup.plot_vars) + 1*5e-2*randn(length(obs_tmp.setup.plot_vars),1);
    end
    
    out.time = time;
    
    obs_tmp.init.X = [];
    obs_tmp.init.X(obs_tmp.init.traj).val = [];

    % true system - correct initial condition and no noise
    % considered
    X = obs_tmp.setup.ode(@(t,x)obs_tmp.setup.model_reference(t, x, obs_tmp.init.params, obs_tmp), time, x0_true ,obs_tmp.init.params.odeset); 
    obs_tmp.init.X(obs_tmp.init.traj).val = X.y;
    out.x_true.val = X.y;            
    
    % reset filter state
    if Nfilt
        obs_tmp.init.X_filter(1).val = cell(obs_tmp.setup.Nfilt,obs_tmp.setup.dim_out);
        for filt=1:Nfilt
           for dim=1:obs_tmp.setup.dim_out
               obs_tmp.init.X_filter(1).val{filt,dim} = zeros(obs_tmp.setup.filterTF(filt).dim,obs_tmp.setup.Niter);
           end
        end
    end
    % reset measurements
    obs_tmp.init.Y_full_story(1).val = zeros(obs_tmp.setup.Nfilt+1,obs_tmp.setup.dim_out,0);
    obs_tmp.init.Ytrue_full_story(1).val = zeros(obs_tmp.setup.Nfilt+1,obs_tmp.setup.dim_out,0);
    
    % set measurements
    % here the noise is noise added aggording to noise_spec
    for i=1:Niter
        y(1).val = [];
        obs_tmp.init.Ytrue_full_story(1).val(1,:,i) = obs_tmp.setup.measure_reference(out.x_true.val(:,i),obs_tmp.init.params,time(i));
        obs_tmp.init.noise_story(1).val(:,i) = obs_tmp.setup.noise*(noise_mu(obs_tmp.setup.observed_state) + noise_std(obs_tmp.setup.observed_state).*randn(obs_tmp.setup.dim_out,1));        
        y(1).val =  reshape(obs_tmp.init.Ytrue_full_story(1).val(1,:,i),obs_tmp.setup.dim_out,1) + obs_tmp.init.noise_story(1).val(:,i);
        obs_tmp.init.Y_full_story(1).val(1,:,i) = y(1).val;
        
        if Nfilt
            tspan_pos = [max(1,i-1), i];
            [dy, x_filter] = obs_tmp.measure_filter(obs_tmp.init.Y_full_story(1).val,tspan_pos,obs_tmp.init.X_filter(1));                
            for filt=1:Nfilt
                for dim=1:obs_tmp.setup.dim_out
                    y_tmp(dim,1) = dy{filt,dim}.val(end);
                    obs_tmp.init.X_filter(1).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                end   
                y(1).val = [y(1).val, y_tmp];
            end
            
            % insert measurements
            for term=1:Nfilt+1                        
                obs_tmp.init.Y_full_story(1).val(term,:,i) = y(1).val(:,term);             
            end
        end
        
        
    end
    
    % get measures
    out.y_true.val = obs_tmp.init.Y_full_story(1).val;

    % integration loop
    for i=1:N
        
        % Display iteration step  
        clc
        disp(['Iteration Number: ', num2str(i),'/',num2str(N)])                                                        

        % real system - initial condition perturbed 
        X = obs_tmp.setup.ode(@(t,x)obs_tmp.setup.model(t, x, obs_tmp.init.params, obs_tmp), time, x0_dist(:,i),obs_tmp.init.params.odeset);
        out.x_dist(i).val = X.y;      

    end        
    
    for i=1:N  
        
        if Nfilt
            out.x_filter(i).val = cell(obs_tmp.setup.Nfilt,obs_tmp.setup.dim_out);
            for filt=1:Nfilt
               for dim=1:obs_tmp.setup.dim_out
                   out.x_filter(i).val{filt,dim} = zeros(obs_tmp.setup.filterTF(filt).dim,obs_tmp.setup.Niter);
               end
            end 
        end
        
        for iter=1:Niter 
            
            y(1).val = [];
            y(1).val = obs_tmp.setup.measure(out.x_dist(i).val(:,iter), obs_tmp.init.params, time(iter));
            out.y_dist(i).val(1,:,iter) = y(1).val;
               
        
            if Nfilt
                
                tspan_pos = [max(1,iter-1), iter];
                [dy, x_filter] = obs_tmp.measure_filter(out.y_dist(i).val,tspan_pos,out.x_filter(i));                
                for filt=1:Nfilt
                    for dim=1:obs_tmp.setup.dim_out
                        y_tmp(dim,1) = dy{filt,dim}.val(end);
                        out.x_filter(i).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                    end   
                    y(1).val = [y(1).val, y_tmp];
                end
                
                % insert measurements
                for term=1:Nfilt+1
                    out.y_dist(i).val(term,:,iter) = y(1).val(:,term);             
                end
            end
        
            
        end
    end
    
    out.obs = obs_tmp;

end



