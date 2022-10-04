%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function [params_s,obs_s,SimParams,params,out] = simulation_multiMHE_realdata(model)

rng default

fast = 1;
slow = 1;
tend = 15000;
Ts = 5;

%%%% Init Section %%%%
[params_s, obs_s, time, SimParams, params, SIMstate, SIMinput, SIMmeasure] = InitMultiMHE(tend,Ts);
obs_fast = obs_s{1};
obs_slow = obs_s{2};
params_fast = params_s{1};
params_slow = params_s{2};

% measure filter
[~, ~, reference] = filter_define(obs_s{1}.setup.Ts,obs_s{1}.setup.Nts);
%%%% measure filter state %%%%
obs_fast.init.x_filter = reference.x0;
obs_slow.init.x_filter = reference.x0;

% SIMULINK
options = simset('SrcWorkspace','current');
out = sim(model,params.Tend,options);
data = out.simout.ECM_Vb_noise.Data';


%% %%%% SIMULATION %%%%
% generate data
%%%% SWITCH Y WITH THE COLLECTED DATA %%%%%%
y = data;
Niter = obs_fast.setup.Niter;
Ntraj = obs_fast.setup.Ntraj;

%%% THIS SHOULD REMAIN IN THE CODE %%%
obs_fast.init.Y_full_story(1).val(1,:,1:size(y,2)) = y;   
obs_slow.init.Y_full_story(1).val(1,:,1:size(y,2)) = y;   
obs_fast.init.Ytrue_full_story(1).val(1,:,1:size(y,2)) = out.simout.ECM_Vb.Data;   
obs_slow.init.Ytrue_full_story(1).val(1,:,1:size(y,2)) = out.simout.ECM_Vb.Data;   
obs_fast.init.input_story.val(1,1:size(y,2)) = out.simout.u.Data;   
obs_slow.init.input_story.val(1,1:size(y,2)) = out.simout.u.Data;   
obs_fast.init.X(1).val(1,1:size(y,2)) = out.simout.ECM_soc.Data;   
obs_slow.init.X(1).val(1,1:size(y,2)) = out.simout.ECM_soc.Data;   

% init estimation data
% obs_fast.init.X_est(1).val(1:2,1) = obs_fast.init.X(1).val(1:2,1).*(0.8 + 0.1*randn(2,1));
% obs_slow.init.X_est(1).val(1:2,1) = obs_fast.init.X_est(1).val(1:2,1);

% start time counter
t0 = tic;

% integration loop
for i = 1:Niter
    
    % Display iteration step
    if ((mod(i,10) == 0) || (i == 1))
        clc
        disp('FAST MHE:')
        disp(['Iteration Number: ', num2str(obs_fast.setup.time(i)),'/',num2str(obs_fast.setup.time(Niter))])
        disp(['Last J: ',num2str(obs_fast.init.Jstory(end))]);
        disp('SLOW MHE:')
        disp(['Iteration Number: ', num2str(obs_slow.setup.time(i)),'/',num2str(obs_slow.setup.time(Niter))])
        disp(['Last J: ',num2str(obs_slow.init.Jstory(end))]);
    end
    
    % set current iteration in the obsopt class
    obs_fast.init.ActualTimeIndex = i;
    obs_fast.init.t = obs_fast.setup.time(i);
    obs_slow.init.ActualTimeIndex = i;
    obs_slow.init.t = obs_fast.setup.time(i);
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate
    for traj = 1:Ntraj
        
        % set trajectories
        obs_fast.init.traj = traj;
        obs_slow.init.traj = traj;
                 
        % propagate only if the time gets over the initial time instant
        if(i > 1)
            
            % define the time span of the integration
            startpos = i-1;
            stoppos = i;
            tspan = time(startpos:stoppos); 
            
            % fast
            X = obs_fast.setup.ode(@(t,x)obs_fast.setup.model(t, x, obs_fast.init.params, obs_fast), tspan, obs_fast.init.X_est(traj).val(:,startpos),params_fast.odeset);
            obs_fast.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];
            
            % slow
            X = obs_slow.setup.ode(@(t,x)obs_slow.setup.model(t, x, obs_slow.init.params, obs_slow), tspan, obs_slow.init.X_est(traj).val(:,startpos),params_slow.odeset);
            obs_slow.init.X_est(traj).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];                                          
        end
        
        %%%% FILTER MEASUREMENT %%%%                 
        % filter on the measurements    
        % reference filter (filterScale)
        if 1
            try
                u = [y(:,i);reshape(obs_fast.init.Y_full_story(traj).val(1,:,i),1,obs_fast.setup.dim_out)];
            catch
                u = [y(:,i);zeros(1,obs_setup.setup.dim_out)];
            end
            [y_filt, ~, x] = lsim(reference.TF,u,obs_fast.setup.tspan,obs_fast.init.x_filter(max(1,i-1)));  
            obs_fast.init.x_filter(:,i) = x(end);
            obs_slow.init.x_filter(:,i) = x(end);
            y_meas(traj).val = transpose(y_filt(end,:));
        else
            y_meas(traj).val = y(:,i);        
        end
                            
    end
       
    %%%% OBSERVER %%%%
    
    % call the observer     
    if fast 
        if obs_slow.init.just_optimised && slow
            obs_fast.init.params = obs_fast.setup.params.params_update(obs_slow.init.params,obs_fast.init.X_est(1).val(:,end));
            pos_index = nonzeros(obs_fast.init.Y_space);
            obs_fast.init.X_est.val(obs_fast.setup.update_vars,pos_index(1):pos_index(end)) = obs_slow.init.X_est.val(obs_fast.setup.update_vars,pos_index(1):pos_index(end));
        end        
        tfast = tic;
        obs_fast = obs_fast.observer(obs_fast.init.X_est,y_meas);            
        obs_fast.init.iter_time(obs_fast.init.ActualTimeIndex) = toc(tfast);
    end
    
    % call the observer      
    if slow
        tslow = tic;
        obs_slow = obs_slow.observer(obs_slow.init.X_est,y_meas);            
        obs_slow.init.iter_time(obs_slow.init.ActualTimeIndex) = toc(tslow);
    end

end
% 
% overall computation time
obs_fast.init.total_time = toc(t0);
obs_slow.init.total_time = obs_fast.init.total_time;
params_s = {params_fast, params_slow};
obs_s = {obs_fast, obs_slow};
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

