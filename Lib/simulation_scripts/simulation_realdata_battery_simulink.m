%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general modelgiaccagli.mattia@gmail.com
% INPUT: none
% OUTPUT: params,obs
function [params,obs] = simulation_realdata_battery_simulink(model_name)

% generate from simulink
params = params_battery_simulink;
options = simset('SrcWorkspace','current');
params.out = sim(model_name,params.time(end),options);
params_sim = params;
clear params

% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw = 30;
Nts = 20;

% noise
rng default

% set sampling time
Ts = params_sim.Ts;

% set initial and final time instant
% remember to set the final time and sampling time accordingly to the data
% that you measured
t0 = params_sim.time(1);
tend = params_sim.time(end);
% uncomment to test the MHE with a single optimisation step
% tend = 1*(Nw*Nts-1)*Ts;

%%%%%% general functions
params_init = @params_battery_tushar;
params_update = @params_update_battery_tushar;
model = @model_battery_tushar;
model_reference = @model_battery_tushar;
measure = @measure_battery_tushar;
measure_reference = @measure_battery_tushar;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% filters %%%%
[filter, filterScale, reference] = filter_define(Ts,Nts);

%%%% integration method %%%%
% ode45-like integration method. For discrete time systems use @odeDD
ode = @odeDD;

%%%% input law %%%
input_law = @control_battery;

%%%% params init %%%%
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',1, 'params_update', params_update, ...
            'model',model,'measure',measure,'ode',ode, 'odeset', [1e-3 1e-6], ...
            'input_enable',1,'input_law',input_law,'params_init',params_init,'addons',params_sim);
             
%%%% observer init %%%%
%%%% define arrival cost %%%%%
terminal_states = params.opt_vars;
terminal_weights = 1e0*ones(size(terminal_states));
% OCV %
terminal_weights([3 7 11]) = 1;
% R0 %
terminal_weights([4 8 12]) = 1e2;
% OCV %
terminal_weights([5 9 13]) = 1e2;
% OCV %
terminal_weights([6 10 14]) = 1e4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% create observer class instance. For more information on the setup
% options check directly the class constructor in obsopt.m
obs = obsopt('DataType', 'simulated', 'optimise', 1, 'MultiStart', params.multistart, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'WaitAllBuffer', 0, 'params',params, 'filters', filterScale,'filterTF', filter, ...
          'model_reference',model_reference, 'measure_reference',measure_reference, ...
          'Jdot_thresh',0.95,'MaxIter', 1, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', Inf, 'AdaptiveParams', [4 80 2 1 10 params.OutDim_compare], ...
          'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @fminsearchcon, 'terminal', 1, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 0, ...
          'ConPos', [], 'LBcon', [], 'UBcon', [],'NONCOLcon',@nonlcon_fcn,'Bounds', 1,'BoundsPos',[1 4 5],'BoundsValLow',[1e-3 1e-3 1e-3],'BoundsValUp',[1 1e3 1e3]);



%% %%%% SIMULATION %%%%
% generate data
%%%% SWITCH Y WITH THE COLLECTED DATA %%%%%%
Niter = obs.setup.Niter;
Ntraj = obs.setup.Ntraj;

% start time counter
t0 = tic;

% integration loop
for i = 1:Niter
    
    % Display iteration step
    if ((mod(i,100) == 0) || (i == 1))
        clc
        disp('MHE:')
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(Niter))])
        disp(['Last J: ',num2str(obs.init.Jstory(end))]);        
    end
    
    % set current iteration in the obsopt class
    obs.init.ActualTimeIndex = i;
    obs.init.t = obs.setup.time(obs.init.ActualTimeIndex);    
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate
    for traj = 1:Ntraj
        
        % set trajectories
        obs.init.traj = traj;
        obs_slow.init.traj = traj;
                 
        % propagate only if the time gets over the initial time instant
        if (obs.init.ActualTimeIndex > 1)
            
            % define the time span of the integration
            startpos = obs.init.ActualTimeIndex-1;
            stoppos = obs.init.ActualTimeIndex;
            tspan = obs.setup.time(startpos:stoppos); 
            
            % fast
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, obs.init.X_est(traj).val(:,startpos),params.odeset);
            obs.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];
                                                      
        end                
        
        %%%% FILTER MEASUREMENT %%%%                 
        % filter on the measurements    
        % reference filter (filterScale)
        y_meas(traj).val = params.y_sim(:,i);            
    end
       
    %%%% OBSERVER %%%%
    
    % call the observer                 
    tfast = tic;
    obs = obs.observer(obs.init.X_est,y_meas);            
    obs.init.iter_time(obs.init.ActualTimeIndex) = toc(tfast);                          

end
% 
% overall computation time
obs.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% post sim operations %%%
obs.init.X.val = zeros(params.dim_state,params.Niter);
obs.init.X.val(1,:) = params_sim.out.simout.ECM_soc.Data';
obs.init.Ytrue_full_story.val = zeros(obs.setup.Nfilt,params.OutDim,params.Niter);
obs.init.Ytrue_full_story.val(1,1,:) = params_sim.out.simout.ECM_Vb.Data';
% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

