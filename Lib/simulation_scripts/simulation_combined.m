%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [params,obs] = simulation_combined

%%%% Init Section %%%%
% uncomment to close previously opened figures
% close all
    
% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw = 40;
Nts = 2;

% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
% tend = 7;
% uncomment to test the MHE with a single optimisation step
tend = 1*(Nw*Nts-1)*Ts;

%%%% filters %%%%
[filter, filterScale] = filter_define(Ts,Nts);

%%%% integration method %%%%
% ode45-like integration method. For discrete time systems use @odeDD
ode = @oderk4_fast;

%%%% input law %%%
input_law = @control;

%%%% measurement noise %%%%
noise_mat = 0*ones(8,2);

%%%% params init - ID%%%%
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',noise_mat, 'params_update', @params_update_control_test_ID, ...
        'model',@model_control_test_ID,'measure',@measure_control_test_ID,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',1,'input_law',input_law,'params_init',@params_control_test_ID);

%%%% observer init - ID %%%%
obs = obsopt('DataType', 'simulated', 'optimise', 1, ... 
      'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'control_design', 0, 'params',params, 'filters', filterScale, ...
      'filterTF', filter, 'Jdot_thresh',0.9,'MaxIter',1000, 'Jterm_store', 0, 'AlwaysOpt', 1 , 'print', 1 , 'SafetyDensity', 3, ...
      'AdaptiveHist', [1e-2, 3e-2, 1e0], 'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminunc, 'spring', 0);

%% %%%% SIMULATION %%%%

% start time counter
t0 = tic;

% integration loop
for i = 1:obs.setup.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(obs.setup.Niter))])
    end
    
    % set current iteration in the obsopt class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(i);
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate
    for traj = 1:obs.setup.Ntraj
                 
        % propagate only if the time gets over the initial time instant
        if(obs.init.ActualTimeIndex > 1)
            
            % define the time span of the integration
            startpos = obs.init.ActualTimeIndex-1;
            stoppos = obs.init.ActualTimeIndex;
            tspan = obs.setup.time(startpos:stoppos);            

            % true system - correct initial condition and no noise
            % considered
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, obs.setup.params, obs), tspan, obs.init.X(traj).val(:,startpos),obs.setup.params.odeset); 
            obs.init.X(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];

            % real system - initial condition perturbed 
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, obs.init.X_est(traj).val(:,startpos),obs.setup.params.odeset);
            obs.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];      
        end
        
        %%%% REAL MEASUREMENT %%%%
        % here the noise is noise added aggording to noise_spec
        obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex) = obs.setup.measure_reference(obs.init.X(traj).val(:,obs.init.ActualTimeIndex),obs.init.params,obs.setup.time(obs.init.ActualTimeIndex));
        obs.init.noise_story(traj).val(:,obs.init.ActualTimeIndex) = obs.setup.noise*(obs.setup.noise_mu(obs.setup.observed_state)  + obs.setup.noise_std(obs.setup.observed_state).*randn(obs.setup.dim_out,1));
        y_meas(traj).val =  reshape(obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex),obs.setup.dim_out,1) + obs.init.noise_story(traj).val(:,obs.init.ActualTimeIndex);
    end
      
    %%%% OBSERVER %%%%
    % start time counter for the observation
    t1 = tic;
    
    % call the observer
    obs = obs.observer(obs.init.X_est,y_meas);
    
    % update the model parameters
%     params = obs.init.params;
    
    % stop time counter for the observer. Each optimisation process is
    % timed.
    obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);

end

%%%% SNR %%%
% the SNR is computed on the mesurements
for traj = 1:obs.setup.Ntraj
    for i=1:obs.setup.dim_out
        obs.init.SNR(traj).val(i) = 10*log(sum(obs.init.Ytrue_full_story(traj).val(1,i,:).^2)/sum(obs.init.noise_story(traj).val(i,:).^2));
    end
end

% overall computation time
obs.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section_control(); 

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

