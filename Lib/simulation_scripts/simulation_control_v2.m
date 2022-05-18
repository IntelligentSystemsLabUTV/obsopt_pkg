function [params,obs] = simulation_control_v2

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close figures
% close all

% init model
    
% init observer buffer
Nw = 100;
Nts = 1;

% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
% tend = 5;
tend = (Nw-1)*Ts;
% tend = 1*(Nw*Nts)*Ts;

%%%%%%%%%%% params function %%%%%%%%%%%
% params function: this file shall be in the following form:
% params = params_function()
% INPUT: no input
% OUTPUT: structure with all the necessary parameter to implement the
% model equations. e.g. 
% params.M = mass
% params.b = friction coefficient
% params.observed_state = [2 4] array defining the state elements which
% are actually observed. This will come useful in the measure function
params_init = @params_double_pendulum_input;
params_update = @params_update_doublependulum_control;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% model function %%%%%%%%%%%
% model function: this file shall be in the following form:
% xdot = model_function(t,x,params) 
% INPUT:
% t = current time instant (used by integration like ode45)
% x = current state
% params = structure with model parameters (see params_init)
% OUTPUT:
% xdot = output of the state space model
model = @model_double_pendulum_default_array;
% model_reference = model;
model_reference = @model_reference;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% measure function %%%%%%%%%%%
% measure function: this file shall be in the following form:   
% y = measure_function(x,params)
% INPUT: 
% x = current state
% params = structure with model parameters (see params_init)
% OUTPUT:
% y = measure (no noise added). In the following examples it holds
% y = x(params.observed_state) (see params_init)
measure = @measure_manipulator_array;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% filters %%%%%%%%
[filter, filterScale,reference ] = filter_define(Ts,Nts);
% set the integration method
ode = @oderk4_fast;

% define the input law used: here it's just for a test. You can also
% comment out this line, a default sinusoidal input is hard coded in
% model_init();
input_law = @control;

% init the parameters structure. The model_init file has lots of setup
% options (varargin). The most important is the 'params_init' option, 
% which takes as input the function handle to the previously defined
% @params_init. For more information see directly the file.
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',[0,0], 'params_update', params_update, ...
        'model',model,'measure',measure,'StateDim',12,'ObservedState',[1 2],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',1,'dim_input',2,'input_law',input_law,'params_init',params_init);    

% create observer class instance. For more information on the setup
% options check directly the class constructor
obs = obsopt_v1103('DataType', 'simulated', 'optimise', 1, ... 
      'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'control_design', 1 , 'model_reference', model_reference, ...    
      'params',params, 'filters', filterScale,'filterTF', filter, 'Jdot_thresh',0.9,'MaxIter',2000,...
      'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 1 , 'SafetyDensity', 3, 'AdaptiveHist', [1e-4, 3e-4, 1e0], ...
      'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminunc);
  
% reference filtering
params.reference = reference;
params.x_filter = zeros(params.dim_state,1);
obs.init.x_filter_reference(:,1) = zeros(params.dim_state,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% integration loop
t0 = tic;
for i = 1:obs.setup.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(obs.setup.Niter))])
        disp(['Last cost function: ', num2str(obs.init.Jstory(end))]);
    end
    
    % set current interation in the class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(i);
    
    %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    for traj = 1:obs.setup.Ntraj        
        
        if(obs.init.ActualTimeIndex > 1)
            
            % input
            drive = obs.drive(obs.init.X(traj).val(:,obs.init.ActualTimeIndex-1),obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex-1)); % state tracking
            params.u = obs.setup.params.input(obs.init.t,drive,obs.init.params);
            obs.init.params.u = params.u;
            
            % true system
            obs.init.t_ode_start = tic;
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, params), obs.setup.tspan, obs.init.X(traj).val(:,obs.init.ActualTimeIndex-1),params.odeset);  
            obs.init.X(traj).val(:,obs.init.ActualTimeIndex) = X.y(:,end);     
            obs.init.t_ode(end+1) = toc(obs.init.t_ode_start);

            % real system
            obs.init.t_ode_start = tic;
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex-1),params.odeset);
            obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex) = X.y(:,end); 
            obs.init.t_ode(end+1) = toc(obs.init.t_ode_start);
        end
        
        %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
        % init measurement
        y = zeros(obs.init.params.dim_state,1);
        y(obs.setup.observed_state) = obs.setup.measure(obs.init.X(traj).val(:,obs.init.ActualTimeIndex),obs.init.params);
        
        % filter output
        if 1
        x = obs.init.x_filter_reference(:,max(1,obs.init.ActualTimeIndex-1));
        u = repmat(y,1,2);        
        x_dot = zeros(obs.init.params.dim_state,1);
        for IterFilter=1:params.dim_state
            [y(IterFilter,1),x_dot(IterFilter,1)] = discreteSS(x(IterFilter),u(IterFilter,:),params.reference.A,params.reference.B,params.reference.C,params.reference.D); 
        end            
        obs.init.x_filter_reference(:,obs.init.ActualTimeIndex) = x_dot;
        end
        
        % here the noise is added
        y = y(obs.setup.observed_state);
        obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex) = y;
        obs.init.noise_story(traj).val(:,obs.init.ActualTimeIndex) = obs.setup.noise*(obs.setup.noise_mu  + obs.setup.noise_std*randn(obs.setup.dim_out,1));
        y_meas(traj).val =  reshape(obs.init.Ytrue_full_story(traj).val(1,:,obs.init.ActualTimeIndex),obs.setup.dim_out,1) + obs.init.noise_story(traj).val(:,obs.init.ActualTimeIndex);
    end
      
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t1 = tic;
    obs = obs.observer(obs.init.X_est,y_meas);
%     params = obs.init.params;
    obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);
    
%     % params update
%     x = obs.init.X_est(1).val(:,obs.init.ActualTimeIndex);
%     params = params_update(params,x);

end
% SNR
for traj = 1:obs.setup.Ntraj
    for i=1:obs.setup.dim_out
        obs.init.SNR(traj).val(i) = 10*log(sum(obs.init.Ytrue_full_story(traj).val(1,i,:).^2)/sum(obs.init.noise_story(traj).val(i,:).^2));
    end
end
% time
obs.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
obs.plot_section_control(); 
% plot_manipulator(obs);

if 0
    load handel
    sound(y,Fs)
end
end

