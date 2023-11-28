%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [obs,params] = simulation_drone

%%%% Init Section %%%%
% uncomment to close previously opened figures
% close all
rng('default');
% rng(42);
% rng(23);
% rng(2);
    
% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw = 400;
Nts = 5;

% set sampling time
Ts = 1e-2; 

% set initial and final time instant
t0 = 0;
%tend = 40;
% uncomment to test the MHE with a single optimisation step
tend = 1*(Nw*Nts-1)*Ts;

% set switching time
Tsw = 10;

%%%% params init function %%%%
params_init = @params_drone;

%%%% params update function %%%%
params_update = @params_update_drone;

%%%% model function %%%%
model = @model_drone;

%%%% model reference function %%%%
model_reference = @model_drone_reference;

%%%% measure function %%%%
measure_reference = @measure_drone_reference;
measure = @measure_drone;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% filters %%%%
[filter, filterScale] = filter_define(Ts,1);

%%%% integration method %%%%
ode = @odeDD;

%%%% input law %%%
input_law = @control_drone;

%%%% params init %%%%
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1, 'params_update', params_update, ...
            'model',model,'measure',measure,'ode',ode, 'odeset', [1e-3 1e-6], ...
            'input_enable',1,'input_law',input_law,'params_init',params_init);
             
%%%% observer init %%%%
% defien arrival cost
terminal_states = params.opt_vars;
terminal_weights = 1e0*ones(size(terminal_states));

% create observer class instance. For more information on the setup
% options check directly the class constructor in obsopt.m
obs = obsopt('DataType', 'simulated', 'optimise',1 , 'print', 1, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, ...
          'PE_maxiter', 0, 'WaitAllBuffer', 1, 'params',params, 'filters', filterScale,'filterTF', filter, ...
          'model_reference',model_reference, 'measure_reference',measure_reference, ...
          'AdaptiveSampling',0, 'AdaptiveParams', [10 20 1 1 0.01 params.pos_uwb_out], ... %?
          'FlushBuffer', 1, ...
          'opt', @fminsearchcon, ...
          'Jdot_thresh',0.95, 'MaxIter', 20, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'SafetyDensity', Inf, ...
          'MultiStart', params.multistart, 'J_normalise', 1, 'MaxOptTime', Inf, ...
          'terminal', 0, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 1, ...
          'ConPos', [], 'LBcon', [], 'UBcon', [],'Bounds', 0 ,'NONCOLcon',@nonlcon_fcn_rover);

%% %%%% SIMULATION %%%%
% obs.init.X.val(params.pos_other,1) = 0;
% obs.init.X_est.val(params.pos_other,1) = 0;
% start time counter
t0 = tic;

% integration loop
for i = 1:obs.setup.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(obs.setup.Niter))])
        disp(['Last J:', num2str(obs.init.Jstory(end))]);
    end


    % set current iteration in the obsopt class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(i);

    % define time span (single integration)
    startpos = max(1,obs.init.ActualTimeIndex-1);
    stoppos = obs.init.ActualTimeIndex;
    tspan = obs.setup.time(startpos:stoppos);
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate    
        
    for traj = 1:obs.setup.Ntraj
        
        % update traj
        obs.init.traj = traj;
             
        % propagate only if the time gets over the initial time instant
        if(obs.init.ActualTimeIndex > 1)                
            
            % true system - correct initial condition and no noise
            % considered                 
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, obs.setup.params, obs), tspan, obs.init.X(traj).val(:,startpos),params.odeset); 
            obs.init.X(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];
    
            % real system - initial condition perturbed             
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, obs.init.X_est(traj).val(:,startpos),params.odeset);
            obs.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)]; 
                      
        end
        
        %%%% REAL MEASUREMENT %%%%
        % here the noise is noise added aggording to noise_spec
            [y_meas(traj).val, obs] = measure_drone_reference(obs.init.X_est(traj).val(:,stoppos),obs.init.params,obs.setup.time(startpos:stoppos),...
                                                                            obs.init.input_story(traj).val(:,max(1,startpos)),obs,Tsw);

            % [tmp, obs] = measure_drone(obs.init.X(traj).val(:,stoppos),obs.init.params,obs.setup.time(startpos:stoppos),...
            %                                                                 obs.init.input_story(traj).val(:,max(1,startpos)),obs);

    end
    
    %%%% MHE OBSERVER (SAVE MEAS) %%%%
    % t1 = tic;    
    if 1
        obs = obs.observer(obs.init.X_est,y_meas);
        % obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);   
        if obs.init.break
            break;
        end
    end
    % obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);                                                
    
    

end

%%%% SNR %%%
% the SNR is computed on the mesurements https://ieeexplore.ieee.org/document/9805818 
for i=1:obs.setup.dim_out
    obs.init.SNR(1).val(i) = 10*log(sum(obs.init.Ytrue_full_story(1).val(1,i,:).^2)/sum(obs.init.noise_story(1).val(i,:).^2));
end


% overall computation time
obs.init.total_time = toc(t0);
fprintf("Overall computation time: %f s\n", toc(t0))
fprintf("\nGamma: \n")
disp(obs.init.params.gamma)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section_control(); 

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

