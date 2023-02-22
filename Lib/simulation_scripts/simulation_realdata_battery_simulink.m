%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general modelgiaccagli.mattia@gmail.com
% INPUT: none
% OUTPUT: params,obs
function [params,obs,SimParams,out] = simulation_realdata_battery_simulink(model_name)

% init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
Nw = 60;
Nts = 5;

% noise
rng default

% set sampling time
Ts = 1e0;

% set initial and final time instant
% remember to set the final time and sampling time accordingly to the data
% that you measured
t0 = 0;
tend = 200;
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
            'input_enable',1,'input_law',input_law,'params_init',params_init);
             
%%%% observer init %%%%
% defien arrival cost
terminal_states = params.opt_vars;
terminal_weights = 1e0*ones(size(terminal_states));

% create observer class instance. For more information on the setup
% options check directly the class constructor in obsopt.m
obs = obsopt('DataType', 'simulated', 'optimise', 0, 'MultiStart', params.multistart, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'WaitAllBuffer', 2, 'params',params, 'filters', filterScale,'filterTF', filter, ...
          'model_reference',model_reference, 'measure_reference',measure_reference, ...
          'Jdot_thresh',0.95,'MaxIter', 5, 'Jterm_store', 1, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', Inf, 'AdaptiveParams', [4 80 2 1 10 params.OutDim_compare], ...
          'AdaptiveSampling',0, 'FlushBuffer', 1, 'opt', @patternsearch, 'terminal', 0, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 1, ...
          'ConPos', [], 'LBcon', [], 'UBcon', [],'Bounds', 0,'NONCOLcon',@nonlcon_fcn_rover);

% SIMULINK
% create the bus to be provided to simulink
% stack = [params.C_n; params.eta; params.input_data.SOC; params.input_data.OCV; params.input_data.R0; params.input_data.R1; params.input_data.C1];
% params.SimParams = timeseries(stack.*ones(size(params.time)),params.time);
obs_tmp.init.X_est.val = zeros(size(obs.init.X_est.val(:,1),1),length(obs.setup.time));
obs_tmp.init.input_story.val = zeros(obs.setup.params.dim_input,length(obs.setup.time)-1);
obs_tmp.init.Yhat_full_story.val = zeros(size(obs.init.X_est.val(:,1),1),1,length(obs.setup.time));
[params.SIMstate, params.SIMinput, params.SIMmeasure] = SaveToSimulink(obs_tmp,params.time,0);
return
options = simset('SrcWorkspace','current');
out = sim(model_name,obs.setup.time(end),options);
data = out.simout.ECM_Vb_noise.Data';


%% %%%% SIMULATION %%%%
% generate data
%%%% SWITCH Y WITH THE COLLECTED DATA %%%%%%
Niter = obs.setup.Niter;
Ntraj = obs.setup.Ntraj;

%%% THIS SHOULD REMAIN IN THE CODE %%%
obs.init.Y_full_story(1).val(1,:,1:length(obs.setup.time)) = data;   
obs.init.Ytrue_full_story(1).val(1,:,1:length(obs.setup.time)) = out.simout.ECM_Vb.Data;   
obs.init.input_story_ref.val(1,1:length(obs.setup.time)) = out.simout.u.Data;   
obs.init.X(1).val(:,1:length(obs.setup.time)) = [out.simout.ECM_soc.Data'; obs.init.X.val(2:end,1).*ones(obs.setup.dim_state-1,Niter)];   
obs.init.X(1).val(3,1:length(obs.setup.time)) = interp1(params.input_data.SOC,params.input_data.OCV_nominal,obs.init.X(1).val(1,1:length(obs.setup.time)));
obs.init.X(1).val(4,1:length(obs.setup.time)) = interp1(params.input_data.SOC,params.input_data.R0_nominal,obs.init.X(1).val(1,1:length(obs.setup.time)));
obs.init.X(1).val(5,1:length(obs.setup.time)) = interp1(params.input_data.SOC,params.input_data.R1_nominal,obs.init.X(1).val(1,1:length(obs.setup.time)));
obs.init.X(1).val(6,1:length(obs.setup.time)) = interp1(params.input_data.SOC,params.input_data.C1_nominal,obs.init.X(1).val(1,1:length(obs.setup.time)));

% start time counter
t0 = tic;

% integration loop
for i = 1:Niter
    
    % Display iteration step
    if ((mod(i,100) == 0) || (i == 1))
        clc
        disp('FAST MHE:')
        disp(['Iteration Number: ', num2str(obs.setup.time(i)),'/',num2str(obs.setup.time(Niter_fast))])
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
            startpos_fast = obs.init.ActualTimeIndex-1;
            stoppos_fast = obs.init.ActualTimeIndex;
            tspan_fast = obs.setup.time(startpos_fast:stoppos_fast); 
            
            % fast
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan_fast, obs.init.X_est(traj).val(:,startpos_fast),params_fast.odeset);
            obs.init.X_est(traj).val(:,startpos_fast:stoppos_fast) = [X.y(:,1),X.y(:,end)];
                                                      
        end                
        
        %%%% FILTER MEASUREMENT %%%%                 
        % filter on the measurements    
        % reference filter (filterScale)
        y_meas(traj).val = data(:,i);            
    end
       
    %%%% OBSERVER %%%%
    
    % call the observer     
    if fast         
        updateflag = obs_slow.init.just_optimised && slow;
        if updateflag  
            obs.init.params = obs.setup.params.params_update(obs.init.params,obs_slow.init.X_est(1).val(:,end));
            pos_index_slow = nonzeros(obs.init.Y_space);
            obs.init.X_est.val(obs.setup.update_vars,pos_index_slow(1):pos_index_slow(end)) = obs_slow.init.X_est.val(obs.setup.update_vars,pos_index_slow(1):pos_index_slow(end));
        end        
        tfast = tic;
        obs = obs.observer(obs.init.X_est,y_meas);            
        obs.init.iter_time(obs.init.ActualTimeIndex) = toc(tfast);
    end                        

end
% 
% overall computation time
obs.init.total_time = toc(t0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% save to simulink
% [SIMstate, SIMinput, SIMmeasure] = SaveToSimulink(obs_s{2},1);
% out = sim(model,params.Tend,options);
% obs_s{1}.init.Y_full_story

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%
% obs self plots
% obs.plot_section_control(); 

% the whole process could be long, why not going for a nap? No worries, 
% this "sounds" like a nice way to wake up. (Uncomment)
% load handel
% sound(y,Fs)
end

