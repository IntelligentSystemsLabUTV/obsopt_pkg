%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [obs,params] = simulation_rover_realdata(data)

%%%% Init Section %%%%
% uncomment to close previously opened figures
% close all
% rng('default');
% rng(42);
% rng(23);
rng(2);

% create measurements
for i=1:length(data)
    D = data(i).val.UWB.';
    p = data(i).val.p.';
    v = zeros(size(p));
    IMU = data(i).val.IMU.';
    Q = data(i).val.q.';
    W = data(i).val.W.';
    Y(i).val = [D; p; v; IMU; Q; W];
    Len(i) = size(Y(i).val,2);
end

% truncate experiments
UpNiter = min(Len);

    
% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nts = 60;
Nw = floor(UpNiter/Nts);

% set sampling time
Ts = mean(diff(data(1).val.time));

% set initial and final time instant
t0 = 0;
% tend = 60;
% uncomment to test the MHE with a single optimisation step
tend = 1*(UpNiter-1)*Ts;

%%%% params init function %%%%
params_init = @params_rover;

%%%% params update function %%%%
params_update = @params_update_rover;

%%%% model function %%%%
model = @model_rover;

%%%% model reference function %%%%
model_reference = @model_rover_reference;

%%%% measure function %%%%
measure = @measure_rover;
measure_reference = @measure_rover_reference;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% filters %%%%
[filter, filterScale] = filter_define(Ts,1);

%%%% integration method %%%%
ode = @odeEuler;

%%%% input law %%%
input_law = @control;

%%%% params init %%%%
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0, 'Ntraj', length(data),'params_update', params_update, ...
            'model',model,'measure',measure,'ode',ode, 'odeset', [1e-3 1e-6], ...
            'input_enable',1,'input_law',input_law,'params_init',params_init);
             
%%%% observer init %%%%
% defien arrival cost
terminal_states = params.opt_vars;
terminal_weights = 1e0*ones(size(terminal_states));

% create observer class instance. For more information on the setup
% options check directly the class constructor in obsopt.m
obs = obsopt('DataType', 'simulated', 'optimise', 1 , 'MultiStart', params.multistart, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_ma0iter', 0, 'WaitAllBuffer', 2, 'params',params, 'filters', filterScale,'filterTF', filter, ...
          'model_reference',model_reference, 'measure_reference',measure_reference, ...
          'Jdot_thresh',0.95,'MaxIter', 6, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 1 , 'SafetyDensity', Inf, 'AdaptiveParams', [10 160 1 1 0.5 params.pos_acc_out(1:2)], ...
          'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @patternsearch, 'terminal', 0, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 1, ...
          'ConPos', [], 'LBcon', [], 'UBcon', [],'Bounds', 0,'NONCOLcon',@nonlcon_fcn_rover);

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

            % real system - initial condition perturbed             
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, obs.init.X_est(traj).val(:,startpos),params.odeset);
            obs.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)]; 

        end
        
        %%%% REAL MEASUREMENT %%%%
        % here the noise is noise added aggording to noise_spec
        y_meas(traj).val = Y(traj).val(:,obs.init.ActualTimeIndex);    
        obs.init.Y_full_story(traj).val(1,:,obs.init.ActualTimeIndex) = y_meas(traj).val;
        obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex) = 0;
        obs.init.noise_story(traj).val(1,:,obs.init.ActualTimeIndex) = 0;
        if mod(obs.init.ActualTimeIndex-1,params.UWB_samp) == 0 
            obs.init.params.p_jump(obs.init.traj).val(:,end+1) = data(traj).val.pjump(max(1,obs.init.ActualTimeIndex),:);
            obs.init.params.p_jump_der(obs.init.traj).val(:,end+1) = 0;
            obs.init.params.q_jump(obs.init.traj).val(:,end+1) = data(traj).val.qjump(max(1,obs.init.ActualTimeIndex),:);
            obs.init.params.UWB_pos(end+1) = obs.init.ActualTimeIndex;
        end
        
    end
    
    %%%% MHE OBSERVER (SAVE MEAS) %%%%
    if params.hyb && 1
        t1 = tic;
        obs = obs.observer(obs.init.X_est,y_meas);
        obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);   
        if obs.init.break
            break;
        end
    end

    %%% test %%%
    obs.init.params.UWB_samp_EKF = obs.init.params.UWB_samp;
    obs.init.params.IMU_samp_EKF = obs.init.params.IMU_samp;    
                                             
    
    

end

%%%% SNR %%%
% the SNR is computed on the mesurements https://ieeexplore.ieee.org/document/9805818 
for i=1:obs.setup.dim_out
    obs.init.SNR(1).val(i) = 10*log(sum(obs.init.Ytrue_full_story(1).val(1,i,:).^2)/sum(obs.init.noise_story(1).val(i,:).^2));
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

