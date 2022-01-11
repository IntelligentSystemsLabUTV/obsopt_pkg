%% REMARK: current implementation only works for dim 2 systems, as you can 
% visualise the results
function [params,obs, out] = simulation_attraction

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close figures
close all

% init observer buffer
Nw = 3;
Nts = 3;
    
% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
tend = 2*(Nts*Nw+1)*Ts;

%%%%%%%%%%% params function %%%%%%%%%%%
params_init = @params_mockup;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% model function %%%%%%%%%%%
model = @model_mockup;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% measure function %%%%%%%%%%%
measure = @measure_general;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set the integration method
ode = @oderk4;

input_law = [];
filter = 0;

params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',[0, 0],...
        'model',model,'measure',measure,'StateDim',2,'ObservedState',[1],'ode',ode,...
        'input_enable',0,'dim_input',1,'input_law',input_law,'params_init',params_init);
    
% params.X_est(:,1) = [0.9; 0.7];

% create observer class instance. For more information on the setup
% options check directly the class constructor
obs = obsopt_general_adaptive_flush('Nw',Nw,'Nts',Nts,'ode',ode,...    
      'params',params, 'filters',[1,filter,0,0],'Jdot_thresh',0.9,'MaxIter',3,...
      'Jterm_store', 0, 'AlwaysOpt',1,'print',0,'SafetyDensity',5,'AdaptiveHist',[5e-3, 1e-2],...
      'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminunc);
  
% define storage for attraction regions
out = cell(1,1);
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
    if(obs.init.ActualTimeIndex > 1)
        % input
        obs.init.params.u = obs.setup.params.input(:,i-1);
        
        % true system
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X(:,obs.init.ActualTimeIndex-1));   
        obs.init.X(:,obs.init.ActualTimeIndex) = X.y(:,end);
        
        % real system
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(:,obs.init.ActualTimeIndex-1));
        obs.init.X_est(:,obs.init.ActualTimeIndex) = X.y(:,end);
        obs.init.X_wrong(:,obs.init.ActualTimeIndex) = X.y(:,end);
    end
    
    
    %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
    % here the noise is added
    y_meas = obs.setup.measure(obs.init.X(:,obs.init.ActualTimeIndex),obs.init.params) + obs.setup.noise*(obs.setup.noise_mu  + obs.setup.noise_std*randn(obs.setup.dim_out,1));
    
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % save data previous of the optimisation
    x0 = obs.init.X;
    x0_est_pre= obs.init.X_est;
    
    % region of attraction analysis
    obs = obs.observer(obs.init.X_est(:,obs.init.BackIterIndex),y_meas);
    
    % save data post optimisation
    x0_est_post = obs.init.X_est;
       
    % save region of attraction only when optimisation is run (in the other
    % instants it doesn't make much sense, there's no difference between
    % pre and post)
    if obs.init.just_optimised 
        % attraction analysis
        % out = System_analysis_fun_general_v2(x0,offset,filter,Nw,Nts,Nsamples,boundperc,params,obs)
        % no offset, no filter, same (Nw,Nts) as the actual optimisation, 
        % [Nsamples, Nasamples], density od of the region gridding
        % region boundaries wr2 x0: MUST BE SYMMETRIC (gradient calculation)
        % params and obs
%         centralval = [1;0.5];        
%         bound = [0.2; 0.2];     
%         range = [centralval-bound, centralval, centralval+bound];
        
        
        centralval = obs.init.X(:,obs.init.BackIterIndex);        
        bound = [0.5; 0.5]; 
        oneval = ones(size(bound));
        range = [centralval.*(oneval-bound), centralval, centralval.*(oneval+bound)];
        out{end+1} = System_analysis_fun_general_v3(x0(:,obs.init.BackIterIndex),0,0,Nw,Nts,[10,10],range,params,obs);
        
        % save data
        out{end}.x0 = x0(:,obs.init.BackIterIndex);
        out{end}.x0_est_pre = x0_est_pre(:,obs.init.BackIterIndex);
        out{end}.x0_est_post = x0_est_post(:,obs.init.BackIterIndex);
    end
    
end
out(1) = [];
obs.init.total_time = toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section(); 
end

