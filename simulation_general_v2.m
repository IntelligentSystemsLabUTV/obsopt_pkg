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
    
    % set sampling time
    Ts = 1e-1;
    
    % set initial and final time instant
    t0 = 0;
    tend = 10;
    
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
    params_init = @params_runaway_gamma;
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
    model = @model_runaway_gamma;
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
    measure = @measure_runaway;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % set the integration method
    ode = @oderk4;

    % init the parameters structure. The model_init file has lots of setup
    % options (varargin). The most important is the 'params_init' option, 
    % which takes as input the function handle to the previously defined
    % @params_init. For more information see directly the file.
    params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1,'noise_spec',[1e-2, 5e-3],...
            'model',model,'measure',measure,'StateDim',4,'ObservedState',2,'ode',ode,...
            'params_init',params_init);
    
    % init observer buffer
    Nw = 10;
    Nts = 3;

    % create observer class instance. For more information on the setup
    % options check directly the class constructor
    obs = obsopt_general_filters('Nw',Nw,'Nts',Nts,'ode',ode,...    
          'params',params, 'filters',[1,1e-1,1,1e-1],'Jdot_thresh',0.9,'MaxIter',100,...
          'AlwaysOpt',0);
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
    if(obs.init.ActualTimeIndex > 1)
        % input
        obs.init.params.u = obs.setup.params.input(i-1);
        
        % true system
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X(:,obs.init.ActualTimeIndex-1));   
        obs.init.X(:,obs.init.ActualTimeIndex) = X.y(:,end);
        
        % real system
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(:,obs.init.ActualTimeIndex-1));
        obs.init.X_est(:,obs.init.ActualTimeIndex) = X.y(:,end);      
    end
    
    
    %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
    % here the noise is added
    y_meas = obs.setup.measure(obs.init.X(:,obs.init.ActualTimeIndex),obs.init.params) + obs.setup.noise*(obs.setup.noise_mu  + obs.setup.noise_std*randn);
    
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    obs = obs.observer(obs.init.X_est(:,obs.init.ActualTimeIndex),y_meas);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
obs.plot_section(); 
end

