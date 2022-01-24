%% REMARK: current implementation only works for dim 2 systems, as you can 
% visualise the results
function [params,obs, out] = simulation_attraction

%% Init Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close figures
close all

% init observer buffer
Nw = 1;
Nts = 10;
    
% set sampling time
Ts = 1e-1;

% set initial and final time instant
t0 = 0;
tend = 1*(Nts*Nw+1)*Ts;

%%%%%%%%%%% params function %%%%%%%%%%%
params_init = @params_oscillator_VDP;
params_update = @params_update_oscillator_VDP;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% model function %%%%%%%%%%%
model = @model_oscillator_VDP;
model_reference = model;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% measure function %%%%%%%%%%%
measure = @measure_general;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% filters %%%%%%%%
% derivative
G = tf([1 0],[0.1 1]);
SS = ss(G);
D = c2d(SS,Ts*Nts);
filterTF{1}.TF = D;
% integral
G = tf(1,[1 0.001]);
SS = ss(G);
D = c2d(SS,Ts*Nts);
filterTF{2}.TF = D;

% set the integration method
ode = @oderk4;

input_law = [];
filter = 0;

params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',[0, 0], 'params_update', params_update, ...
        'model',model,'measure',measure,'StateDim',2,'ObservedState',[1],'ode',ode,...
        'input_enable',0,'dim_input',2,'input_law',input_law,'params_init',params_init);

% create observer class instance. For more information on the setup
% options check directly the class constructor
obs = obsopt_general_adaptive_flush_filterSS('DataType', 'simulated', 'optimise', 1, ... 
      'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'control_design', 0 , 'model_reference', model_reference, ...    
      'params',params, 'filters', [1,filter,0,0],'filterTF', filterTF, 'Jdot_thresh',0.9,'MaxIter',2,...
      'Jterm_store', 0, 'AlwaysOpt', 0 , 'print', 0 , 'SafetyDensity', 10, 'AdaptiveHist', [2e-2, 4e-2, 5e-7], ...
      'AdaptiveSampling',0, 'FlushBuffer', 1, 'Jterm_store', 0, 'opt', @fminunc);
  
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
    % save data previous of the optimisation
    x0 = obs.init.X(1).val;
    x0_est_pre= obs.init.X_est(1).val;
    
    obs = obs.observer(obs.init.X_est,y_meas);
    
    % params update
    x = obs.init.X_est(1).val(:,obs.init.ActualTimeIndex);
    params = params_update(params,x);
       
    % save region of attraction only when optimisation is run (in the other
    % instants it doesn't make much sense, there's no difference between
    % pre and post)
    if 1 && (obs.init.just_optimised) && (traj==1)
        % attraction analysis
        % out = System_analysis_fun_general_v2(x0,offset,filter,Nw,Nts,Nsamples,boundperc,params,obs)
        % no offset, no filter, same (Nw,Nts) as the actual optimisation, 
        % [Nsamples, Nasamples], density od of the region gridding
        % region boundaries wr2 x0: MUST BE SYMMETRIC (gradient calculation)
        
        % save data post optimisation
        x0_est_post = obs.init.X_est(traj).val;
        
        % params and obs
%         centralval = [1;0.5];        
%         bound = [0.2; 0.2];     
%         range = [centralval-bound, centralval, centralval+bound];
        
       
        centralval = obs.init.X(traj).val(:,obs.init.BackIterIndex);        
        bound = [0.1; 0.1]; 
        oneval = ones(size(bound));
        range = [centralval.*(oneval-bound), centralval, centralval.*(oneval+bound)];
        out{end+1} = System_analysis_fun_general_v3(x0(:,obs.init.BackIterIndex),0,0,Nw,Nts,[25,25],range,params,obs);
        
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

