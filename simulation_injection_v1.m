function [params,obs] = simulation_injection_v1

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close figures
% close all

% init model
    
% init observer buffer
Nw = 5;
Nts = 5;

% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
tend = 7;
%     tend = 1*(Nw*Nts+1)*Ts;

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
params_init = @params_oscillator_VDP;
params_update = @params_update_oscillator_VDP;
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
model = @model_oscillator_VDP_inject;
model_reference = model;
%     model_reference = @model_reference;
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
measure = @measure_general;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% filters %%%%%%%%
[filter, filterScale] = filter_define(Ts,Nts);
% set the integration method
ode = @oderk4;

% define the input law used: here it's just for a test. You can also
% comment out this line, a default sinusoidal input is hard coded in
% model_init();
input_law = @inject;

% init the parameters structure. The model_init file has lots of setup
% options (varargin). The most important is the 'params_init' option, 
% which takes as input the function handle to the previously defined
% @params_init. For more information see directly the file.
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',[0, 0], 'params_update', params_update, ...
        'model',model,'measure',measure,'StateDim',2,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
        'input_enable',1,'dim_input',2,'input_law',input_law,'params_init',params_init);

% create observer class instance. For more information on the setup
% options check directly the class constructor
obs = obsopt_general_inject_v1('DataType', 'simulated', 'optimise', 0, ... 
      'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'control_design', 0 , 'inject_design', 1,  'model_reference', model_reference, ...    
      'params',params, 'filters', filterScale,'filterTF', filter, 'Jdot_thresh',0.9,'MaxIter',40,...
      'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', 3, 'AdaptiveHist', [1e-4, 3e-4, 1e0], ...
      'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminsearch);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% integration loop
t0 = tic;
for i = 1:obs.setup.Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(obs.setup.Niter))])
    end
    
    % set current interation in the class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(i);
    
    %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    for traj = 1:obs.setup.Ntraj
        
        if(obs.init.ActualTimeIndex > 1)
            % input
            y_now = obs.init.Y_full_story(traj).val(1,:,obs.init.ActualTimeIndex-1);
            yhat_now = obs.init.Yhat_full_story(traj).val(1,:,obs.init.ActualTimeIndex-1);           
            params.u = obs.setup.params.input(obs.init.t,y_now,yhat_now,params);
            obs.init.params.u = params.u;

            % true system
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, 0, 0, params), obs.setup.tspan, obs.init.X(traj).val(:,obs.init.ActualTimeIndex-1),params.odeset);   
            obs.init.X(traj).val(:,obs.init.ActualTimeIndex) = X.y(:,end);

            % real system            
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, y_now, yhat_now, obs.init.params), obs.setup.tspan, obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex-1),params.odeset);
            obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex) = X.y(:,end);      
        end
        
        %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
        % here the noise is added
        y_meas(traj).val = obs.setup.measure(obs.init.X(traj).val(:,obs.init.ActualTimeIndex),obs.init.params) + obs.setup.noise*(obs.setup.noise_mu  + obs.setup.noise_std*randn(obs.setup.dim_out,1));
    end
      
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t1 = tic;
    obs = obs.observer(obs.init.X_est,y_meas);
    obs.init.iter_time(obs.init.ActualTimeIndex) = toc(t1);
    
%     % params update
%     x = obs.init.X_est(1).val(:,obs.init.ActualTimeIndex);
%     params = params_update(params,x);

end
obs.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
obs.plot_section_control(); 
end

