%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [params_ID,params_control,obs_ID,obs_control] = simulation_combined

%%%% Init Section %%%%
% uncomment to close previously opened figures
% close all
rng default
    
% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw_ID = 30;
Nts_ID = 10;

Nw_control = 60;
Nts_control = 10;


% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
% tend = 7;
% uncomment to test the MHE with a single optimisation stephttps://in.bgu.ac.il/en/robotics/Pages/default.aspx
tend = 1*(Nw_control*Nts_control-1)*Ts;

%%%% filters %%%%
[filter_ID, filterScale_ID] = filter_define_ID(Ts,Nts_ID);
[filter, filterScale] = filter_define(Ts,Nts_control);

%%%% integration method %%%%
% ode45-like integration method. For discrete time systems use @odeDD
ode = @oderk4_fast;

%%%% measurement noise %%%%
noise_mat_ID = 0*ones(8,2);
noise_mat = 0*ones(6,2);

%%%% params init - ID %%%%
params_ID = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',noise_mat_ID, 'params_update', @params_update_control_test_ID, ...
        'model',@model_control_test_ID,'measure',@measure_control_test_ID,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',1,'input_law',@control_ID,'params_init',@params_control_test_ID);
    
%%%% params init - control %%%%
params_control = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',noise_mat, 'params_update', @params_update_control_test, ...
        'model',@model_control_test,'measure',@measure_control_test,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',1,'input_law',@control,'params_init',@params_control_test);

%%%% observer init - ID %%%%
obs_ID = obsopt('DataType', 'simulated', 'optimise', 0, 'GlobalSearch', 0, 'MultiStart', 0, ... 
      'Nw', Nw_ID, 'Nts', Nts_ID, 'ode', ode, 'PE_maxiter', 0, 'control_design', 0, 'params',params_ID, 'filters', filterScale_ID, 'WaitAllBuffer', 0, ...
      'filterTF', filter_ID, 'Jdot_thresh',0.9,'MaxIter',300, 'Jterm_store', 0, 'AlwaysOpt', 1 , 'print', 1 , 'SafetyDensity', 3, ...
      'AdaptiveHist', [1e-2, 3e-2, 1e0], 'AdaptiveSampling',1, 'FlushBuffer', 1, 'opt', @fminunc, 'spring', 0);
  
%%%% observer init - control %%%%
obs_control = obsopt('DataType', 'simulated', 'optimise', 1, 'GlobalSearch', 0, 'MultiStart', 0, 'J_normalise', 0, ...
      'model_reference', @model_reference, 'measure_reference', @measure_control_ref, ... 
      'Nw', Nw_control, 'Nts', Nts_control, 'ode', ode, 'PE_maxiter', 0, 'control_design', 1, 'params',params_control, 'filters', filterScale, ...
      'filterTF', filter, 'Jdot_thresh',0.9,'MaxIter',50, 'Jterm_store', 0, 'AlwaysOpt', 1 , 'print', 1 , 'SafetyDensity', 3, ...
      'AdaptiveHist', [1e-2, 3e-2, 1e0], 'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminunc, 'spring', 0);

%% %%%% SIMULATION %%%%

% start time counter
t0 = tic;

max_iter = max(obs_ID.setup.Niter,obs_control.setup.Niter);

% integration loop
for i = 1:max_iter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs_ID.setup.time(i)),'/',num2str(obs_ID.setup.time(obs_ID.setup.Niter))])
    end
    
    % set current iteration in the obsopt class
    obs_ID.init.ActualTimeIndex = i;
    obs_ID.init.t = obs_ID.setup.time(i);
    obs_control.init.ActualTimeIndex = i;
    obs_control.init.t = obs_ID.setup.time(i);
    
    %%%% PROPAGATION %%%%
    if (obs_ID.init.ActualTimeIndex <= obs_ID.setup.Niter)
        % forward propagation of the previous estimate
        for traj = 1:obs_ID.setup.Ntraj

            % propagate only if the time gets over the initial time instant
            if(obs_ID.init.ActualTimeIndex > 1)

                % define the time span of the integration
                startpos = obs_ID.init.ActualTimeIndex-1;
                stoppos = obs_ID.init.ActualTimeIndex;
                tspan = obs_ID.setup.time(startpos:stoppos);            

                %%% ID SYSTEM %%%
                % true system - correct initial condition and no noise
                % considered
                X = obs_ID.setup.ode(@(t,x)obs_ID.setup.model_reference(t, x, obs_ID.setup.params, obs_ID), tspan, obs_ID.init.X(traj).val(:,startpos),obs_ID.setup.params.odeset); 
                obs_ID.init.X(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];

                % real system - initial condition perturbed 
                X = obs_ID.setup.ode(@(t,x)obs_ID.setup.model(t, x, obs_ID.init.params, obs_ID), tspan, obs_ID.init.X_est(traj).val(:,startpos),obs_ID.setup.params.odeset);
                obs_ID.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];      

            end

            %%%% REAL MEASUREMENT - ID %%%%
            % here the noise is noise added aggording to noise_spec
            obs_ID.init.Ytrue_full_story(traj).val(1,:,obs_ID.init.ActualTimeIndex) = obs_ID.setup.measure_reference(obs_ID.init.X(traj).val(:,obs_ID.init.ActualTimeIndex),obs_ID.init.params,obs_ID.setup.time(obs_ID.init.ActualTimeIndex));
            obs_ID.init.noise_story(traj).val(:,obs_ID.init.ActualTimeIndex) = obs_ID.setup.noise*(obs_ID.setup.noise_mu(obs_ID.setup.observed_state)  + obs_ID.setup.noise_std(obs_ID.setup.observed_state).*randn(obs_ID.setup.dim_out,1));
            y_meas_ID(traj).val =  reshape(obs_ID.init.Ytrue_full_story(traj).val(1,:,obs_ID.init.ActualTimeIndex),obs_ID.setup.dim_out,1) + obs_ID.init.noise_story(traj).val(:,obs_ID.init.ActualTimeIndex);

        end
        
        %%%% OBSERVER %%%%
        % start time counter for the observation
        t1 = tic;

        % call the observer
        obs_ID = obs_ID.observer(obs_ID.init.X_est,y_meas_ID);
        
        % stop time counter for the observer. Each optimisation process is
        % timed.
        obs_ID.init.iter_time(obs_ID.init.ActualTimeIndex) = toc(t1);
        
    end
    
    if (obs_control.init.ActualTimeIndex <= obs_control.setup.Niter)
        for traj = 1:obs_control.setup.Ntraj

            % propagate only if the time gets over the initial time instant
            if(obs_control.init.ActualTimeIndex > 1)

                % define the time span of the integration
                startpos = obs_control.init.ActualTimeIndex-1;
                stoppos = obs_control.init.ActualTimeIndex;
                tspan = obs_control.setup.time(startpos:stoppos); 

                %%% CONTROL SYSTEM %%%
                % true system - correct initial condition and no noise
                % considered
                X = obs_control.setup.ode(@(t,x)obs_control.setup.model_reference(t, x, obs_control.setup.params, obs_control), tspan, obs_control.init.X(traj).val(:,startpos),obs_control.setup.params.odeset); 
                obs_control.init.X(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];

                % real system - initial condition perturbed 
                X = obs_control.setup.ode(@(t,x)obs_control.setup.model(t, x, obs_control.init.params, obs_control), tspan, obs_control.init.X_est(traj).val(:,startpos),obs_control.setup.params.odeset);
                obs_control.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];                        

            end

            %%%% REAL MEASUREMENT - CONTROL %%%%
            % here the noise is noise added aggording to noise_spec
            obs_control.init.Ytrue_full_story(traj).val(1,:,obs_control.init.ActualTimeIndex) = obs_control.setup.measure_reference(obs_control.init.X(traj).val(:,obs_control.init.ActualTimeIndex),obs_control.init.params,obs_control.setup.time(obs_control.init.ActualTimeIndex));
            obs_control.init.noise_story(traj).val(:,obs_control.init.ActualTimeIndex) = obs_control.setup.noise*(obs_control.setup.noise_mu(obs_control.setup.observed_state)  + obs_control.setup.noise_std(obs_control.setup.observed_state).*randn(obs_control.setup.dim_out,1));
            y_meas_control(traj).val =  reshape(obs_control.init.Ytrue_full_story(traj).val(1,:,obs_control.init.ActualTimeIndex),obs_control.setup.dim_out,1) + obs_control.init.noise_story(traj).val(:,obs_control.init.ActualTimeIndex);
        end
        
        %%%% OBSERVER %%%%
        % start time counter for the observation
        t1 = tic;
        
        obs_control.init.params = obs_ID.setup.params.params_update(obs_control.init.params,obs_ID.init.X_est(1).val(:,end));
        obs_control = obs_control.observer(obs_control.init.X_est,y_meas_control);
        
        % stop time counter for the observer. Each optimisation process is
        % timed.
        obs_control.init.iter_time(obs_control.init.ActualTimeIndex) = toc(t1);
    end


end

%%%% SNR %%%
% the SNR is computed on the mesurements
for traj = 1:obs_ID.setup.Ntraj
    for i=1:obs_ID.setup.dim_out
        obs_ID.init.SNR(traj).val(i) = 10*log(sum(obs_ID.init.Ytrue_full_story(traj).val(1,i,:).^2)/sum(obs_ID.init.noise_story(traj).val(i,:).^2));
    end
end

% overall computation time
obs_ID.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section_control(); 

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

