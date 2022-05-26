%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [params,obs] = simulation_general_v3

%%%% Init Section %%%%
% uncomment to close previously opened figures
% close all
    
% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw = 7;
Nts = 15;

% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
tend = 10;
% uncomment to test the MHE with a single optimisation step
% tend = 1*(Nw*Nts+1)*Ts;

%%%% params init function %%%%
% function: this function shall be in the following form:
% params = params_model()
% INPUT: no input
% OUTPUT: 
% params: structure with all the necessary parameter to implement the
% model equations. e.g. for a mechanical system
% params.M = mass
% params.b = friction coefficient
params_init = @params_oscillator_VDP;

%%%% params update function %%%%
% remark: this file is used if the MHE is set to estimate mode parameters
% too. 
% function: this file shall be in the following form:
% params = params_update(params,x)
% INPUT: 
% params: structure with all the model parameters (see @params_model)
% x: state vector
% OUTPUT: 
% params_out: updated structure with the new model parameters 
params_update = @params_update_oscillator_VDP;


%%%% model function %%%%
% function: this file shall be in the following form:
% xdot = model_function(t,x,params,obs) 
% INPUT:
% t: time instant (used by integration like ode45)
% x: current state
% params: structure with model parameters (see params_init)
% obs: instance of the obsopt observer class
% OUTPUT:
% xdot:output of the state space model
model = @model_oscillator_VDP;

%%%% model reference function %%%%
% remark: !DEVEL! this function is used to generate the reference
% trajectory when the MHE is used in control design. Default considers no
% dynamics, namely simple output tracking. If an MRAC is considered then
% @model should be used.
% function: this file shall be in the same form as @model_function
% INPUT:
% t: time instant (used by integration like ode45)
% x: current state
% params: structure with model parameters (see params_init)
% obs: instance of the obsopt observer class
% OUTPUT:
% xdot:output of the state space model
model_reference = @model_reference;
% model_reference = model;

%%%% measure function %%%%
% function: this file shall be in the following form:   
% y = measure_function(x,params,t)
% INPUT: 
% x = current state
% params = structure with model parameters (see params_init)
% t: time instant
% OUTPUT:
% y = measure (no noise added). In the following examples it holds
% y = x(params.observed_state) (see params_init options)
measure = @measure_general;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% filters %%%%
% remark: this function defines the output filtering applied on the
% measurements. This is fundamental for the filtered MHE (see 
% https://doi.org/10.48550/arXiv.2204.09359)
% function: this file shall be in the following form:   
% [filter, filterScale] = filter_define(Ts,Nts)
% INPUT: 
% Ts: sampling time
% Nts: down-sampling (see https://doi.org/10.48550/arXiv.2204.09359)
% OUTPUT:
% filter: structure with the filter transfer function and ss realization
% filterScale: array weighting the filters in the cost function
[filter, filterScale] = filter_define(Ts,Nts);

%%%% integration method %%%%
% ode45-like integration method. For discrete time systems use @odeDD
ode = @oderk4_fast;

%%%% input law %%%
% function: defines the input law used. Remember to check the @model
% function to correctly use the inputs. 
% INPUT:
% t: time instant
% drive: error variable
% params: structure with model parameters (see params_init)
% OUTPUT:
% u: control variable
input_law = @control;

%%%% measurement noise %%%%
% this should be a vector with 2 columns and as many rows as the state
% dimension. All the noise are considered as Gaussian distributed. The 
% first column defines the mean while the second column the variance.
noise_mat = 1*[0,5e-1; ...
             0, 5e-2; ...
             0, 0];

%%%% params init %%%%
% init the parameters structure through funtion @model_init. 
% The model_init file has lots of setup options (varargin). The most 
% important is the 'params_init' option, which takes as input the function 
% handle to the previously defined @params_init. For more information see 
% directly the model_init.m file.
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',noise_mat, 'params_update', params_update, ...
        'model',model,'measure',measure,'ObservedState',[2],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',0,'input_law',input_law,'params_init',params_init);

%%%% observer init %%%%
% create observer class instance. For more information on the setup
% options check directly the class constructor in obsopt.m
obs = obsopt('DataType', 'simulated', 'optimise', 1, ... 
      'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'control_design', 0 , 'model_reference', model_reference, ...    
      'params',params, 'filters', filterScale,'filterTF', filter, 'Jdot_thresh',0.9,'MaxIter',60,...
      'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 1 , 'SafetyDensity', 3, 'AdaptiveHist', [1e-2, 3e-2, 1e0], ...
      'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminunc, 'spring', 0, 'LBcon', [-Inf -Inf 0]);

%% %%%% SIMULATION %%%%
% remark: the obs.setup.Ntraj variable describes on how many different
% trajectories the MHE is run. This makes sense in the control design
% framework, which is under development. If standard MHE is to be used,
% please ignore obs.setup.Ntraj as it is set to 1.

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
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, params, obs), tspan, obs.init.X(traj).val(:,startpos),params.odeset); 
            obs.init.X(traj).val(:,startpos:stoppos) = X.y;

            % real system - initial condition perturbed 
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, obs.init.X_est(traj).val(:,startpos),params.odeset);
            obs.init.X_est(traj).val(:,startpos:stoppos) = X.y;      
        end
        
        %%%% REAL MEASUREMENT %%%%
        % here the noise is noise added aggording to noise_spec
        obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex) = obs.setup.measure(obs.init.X(traj).val(:,obs.init.ActualTimeIndex),obs.init.params,obs.init.ActualTimeIndex);
        obs.init.noise_story(traj).val(:,obs.init.ActualTimeIndex) = obs.setup.noise*(obs.setup.noise_mu(obs.setup.observed_state)  + obs.setup.noise_std(obs.setup.observed_state).*randn(obs.setup.dim_out,1));
        y_meas(traj).val =  reshape(obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex),obs.setup.dim_out,1) + obs.init.noise_story(traj).val(:,obs.init.ActualTimeIndex);
    end
      
    %%%% OBSERVER %%%%
    % start time counter for the observation
    t1 = tic;
    
    % call the observer
    obs = obs.observer(obs.init.X_est,y_meas);
    
    % update the model parameters
    params = obs.init.params;
    
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
obs.plot_section_control(); 

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

