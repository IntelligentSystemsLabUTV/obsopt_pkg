function [params,obs] = realdata_general_v1(data,time,step)

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
    tmp = diff(time);
    Ts = step*tmp(1);
    
    % set initial and final time instant
    t0 = time(1);
    tend = time(end);
    
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
    params_init = @params_runaway_all;
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
    model = @model_runaway_all;
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % set the integration method
    ode = @oderk4;

    % define the input law used: here it's just for a test. You can also
    % comment out this line, a default sinusoidal input is hard coded in
    % model_init();
    % input_law = [0.3; 0.0067].*ones(2,length(t0:Ts:tend));
    
    % init the parameters structure. The model_init file has lots of setup
    % options (varargin). The most important is the 'params_init' option, 
    % which takes as input the function handle to the previously defined
    % @params_init. For more information see directly the file.
    params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',[0, 0],...
            'model',model,'measure',measure,'StateDim',8,'ObservedState',[2],'ode',ode,...
            'input_enable',0,'dim_input',1,'input_law',[],'params_init',params_init);
    
    % init observer buffer
    Nw = 7;
    Nts = 3;

    % create observer class instance. For more information on the setup
    % options check directly the class constructor
    obs = obsopt_general_adaptive_flush('Nw',Nw,'Nts',Nts,'ode',ode,...    
          'params',params, 'filters',[1,1*5e1,1*1e-1,1*5e-1],'Jdot_thresh',0.9,'MaxIter',200,...
          'AlwaysOpt',0,'print',0,'SafetyDensity',4,'AdaptiveHist',[5e-1, 9e-1],...
          'AdaptiveSampling',1, 'FlushBuffer', 0, 'Jterm_store',0, 'opt', @fmincon);
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
    if ((mod(i,1) == 0) || (i == 1))
        clc
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(obs.setup.Niter))])
        disp(['Last cost function: ', num2str(obs.init.Jstory(end))]);
        try
            disp(['Last dJ condition: ', num2str(obs.init.dJ_cond_story(end))]);
        catch
        end
    end
    
    % set current interation in the class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(i);
    
    %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    if(obs.init.ActualTimeIndex > 1)
        % input
        obs.init.params.u = obs.setup.params.input(:,i-1);
        
        % real system
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(:,obs.init.ActualTimeIndex-1));
        obs.init.X_est(:,obs.init.ActualTimeIndex) = X.y(:,end);      
    end
    
    
    %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
    % here the noise is added
    y_meas = data(min(length(time),obs.init.ActualTimeIndex*step));
    
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    obs = obs.observer(obs.init.X_est(:,obs.init.ActualTimeIndex),y_meas);

end
obs.init.total_time = toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section(); 
end

