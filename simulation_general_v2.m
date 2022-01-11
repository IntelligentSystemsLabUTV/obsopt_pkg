function [params,obs] = simulation_general_v2

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close figures
close all

% if set to 1, the default example will be run. See the obs class
% constructor for more information
default = 0;

% init model
if ~default
    
    % init observer buffer
    Nw = 3;
    Nts = 3;
    
    % set sampling time
    Ts = 1e-1;
    
    % set initial and final time instant
    t0 = 0;
    tend = 5;
%     tend = (Nw*Nts+1)*Ts;
    
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
    params_init = @params_Astolfi_Unit;
    params_update = @params_update_Astolfi_Unit;
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
    model = @model_Astolfi_Unit;
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
    
    % set the integration method
    ode = @oderk4;

    % define the input law used: here it's just for a test. You can also
    % comment out this line, a default sinusoidal input is hard coded in
    % model_init();
    input_law = @control;
    
    % init the parameters structure. The model_init file has lots of setup
    % options (varargin). The most important is the 'params_init' option, 
    % which takes as input the function handle to the previously defined
    % @params_init. For more information see directly the file.
    params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',[0, 0], 'params_update', params_update, ...
            'model',model,'measure',measure,'StateDim',2,'ObservedState',[1],'ode',ode,...
            'input_enable',0,'dim_input',2,'input_law',input_law,'params_init',params_init);

    % create observer class instance. For more information on the setup
    % options check directly the class constructor
    obs = obsopt_general_adaptive_flush('DataType', 'simulated', 'optimise', 1, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'control_design', 0 , 'model_reference', model_reference, ...    
          'params',params, 'filters', [1,1e-1,0,0],'Jdot_thresh',0.9,'MaxIter',1,...
          'Jterm_store', 1, 'AlwaysOpt', 0 , 'print', 0 , 'SafetyDensity', 5, 'AdaptiveHist', [5e-3, 1e-2], ...
          'AdaptiveSampling', 0, 'FlushBuffer', 1, 'Jterm_store', 0, 'opt', @fminunc);
      
else
    % default example, no parameters needed as everything is hard coded in
    % the class constructor
    params = [];

    % class instance creation
    obs = obsopt_general_filters('filters',[1 1e-1 1e-1, 0]);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% integration loop
tic
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
            if obs.setup.control_design == 0
                obs.init.params.u = obs.setup.params.input(obs.init.X(traj).val(:,obs.init.ActualTimeIndex-1),params);
            else
                obs.init.params.u = obs.setup.params.input(obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex-1),params);
            end

            % true system
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, obs.init.params), obs.setup.tspan, obs.init.X(traj).val(:,obs.init.ActualTimeIndex-1));   
            obs.init.X(traj).val(:,obs.init.ActualTimeIndex) = X.y(:,end);

            % real system
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex-1));
            obs.init.X_est(traj).val(:,obs.init.ActualTimeIndex) = X.y(:,end);      
        end
        
        %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
        % here the noise is added
        y_meas(traj).val = obs.setup.measure(obs.init.X(traj).val(:,obs.init.ActualTimeIndex),obs.init.params) + obs.setup.noise*(obs.setup.noise_mu  + obs.setup.noise_std*randn(obs.setup.dim_out,1));
    end
      
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    obs = obs.observer(obs.init.X_est,y_meas);
    
    % params update
    x = obs.init.X_est(1).val(:,obs.init.ActualTimeIndex);
    params = params_update(params,x);

end
obs.init.total_time = toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
obs.plot_section(); 
end

