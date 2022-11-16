%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general modelgiaccagli.mattia@gmail.com
% INPUT: none
% OUTPUT: params,obs
function [params_s,obs_s,SimParams,params,out] = simulation_multiMHE_realdata(model)

rng default

% MHE
fast = 0;
slow = 1;
update_slow = 0;
update_fast = 0;

% time setup
tstart = 0;
tend = 10000;
Ts_fast = 1e0;
Ts_slow = 1e0;
conversion = Ts_slow/Ts_fast;
Ts_slower = min(Ts_fast,Ts_slow);
time_tot_fast = 0:Ts_fast:tend;
time_tot_slow = 0:Ts_slow:tend;

%%%% Init Section %%%%
[params_s, obs_s, time, SimParams, params, SIMstate, SIMinput, SIMmeasure] = InitMultiMHE(tstart,tend,[Ts_fast,Ts_slow]);
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
y_init = data;
Niter_fast = obs_fast.setup.Niter;
Niter_slow = obs_slow.setup.Niter;
Niter = max(Niter_fast,Niter_slow);
Ntraj = obs_fast.setup.Ntraj;

%%% THIS SHOULD REMAIN IN THE CODE %%%
startpos_fast = find(time_tot_fast==tstart);
obs_fast.init.Y_full_story(1).val(1,:,1:length(params_s{1}.time)) = y(startpos_fast:end);   
obs_fast.init.Ytrue_full_story(1).val(1,:,1:length(params_s{1}.time)) = out.simout.ECM_Vb.Data(startpos_fast:end);   
obs_fast.init.input_story.val(1,1:length(params_s{1}.time)) = out.simout.u.Data(startpos_fast:end);   
obs_fast.init.X(1).val(:,1:length(params_s{1}.time)) = [out.simout.ECM_soc.Data(startpos_fast:end)'; params_s{1}.X0(2:end).*ones(obs_fast.setup.dim_state-1,Niter_fast)];   

startpos_slow = find(time_tot_slow==tstart);
obs_slow.init.Y_full_story(1).val(1,:,1:length(params_s{2}.time)) = y(startpos_slow:conversion:end);   
obs_slow.init.Ytrue_full_story(1).val(1,:,1:length(params_s{2}.time)) = out.simout.ECM_Vb.Data(startpos_slow:conversion:end);   
obs_slow.init.input_story.val(1,1:length(params_s{2}.time)) = out.simout.u.Data(startpos_slow:conversion:end);   
obs_slow.init.X(1).val(:,1:length(params_s{2}.time)) = [out.simout.ECM_soc.Data(startpos_slow:conversion:end)'; params_s{1}.X0(2:end).*ones(obs_fast.setup.dim_state-1,Niter_slow)];
obs_slow.init.X(1).val(3,1:length(params_s{2}.time)) = interp1(params.input_data.SOC,params.input_data.OCV_nominal,obs_slow.init.X(1).val(1,1:length(params_s{2}.time)));
obs_slow.init.X(1).val(4,1:length(params_s{2}.time)) = interp1(params.input_data.SOC,params.input_data.R0_nominal,obs_slow.init.X(1).val(1,1:length(params_s{2}.time)));
obs_slow.init.X(1).val(5,1:length(params_s{2}.time)) = interp1(params.input_data.SOC,params.input_data.R1_nominal,obs_slow.init.X(1).val(1,1:length(params_s{2}.time)));
obs_slow.init.X(1).val(6,1:length(params_s{2}.time)) = interp1(params.input_data.SOC,params.input_data.C1_nominal,obs_slow.init.X(1).val(1,1:length(params_s{2}.time)));

% start time counter
t0 = tic;

% integration loop
for i = 1:Niter
    
    % Display iteration step
    if ((mod(i,100) == 0) || (i == 1))
        clc
        disp('FAST MHE:')
        disp(['Iteration Number: ', num2str(obs_fast.setup.time(i)),'/',num2str(obs_fast.setup.time(Niter_fast))])
        disp(['Last J: ',num2str(obs_fast.init.Jstory(end))]);
        disp('SLOW MHE:')
        pos_slow = max(1,floor(i/conversion));
        disp(['Iteration Number: ', num2str(obs_slow.setup.time(pos_slow)),'/',num2str(obs_slow.setup.time(Niter_slow))])
        disp(['Last J: ',num2str(obs_slow.init.Jstory(end))]);
    end
    
    % set current iteration in the obsopt class
    obs_fast.init.ActualTimeIndex = i;
    obs_fast.init.t = obs_fast.setup.time(obs_fast.init.ActualTimeIndex);
    obs_slow.init.ActualTimeIndex = max(1,ceil(i/conversion));
    obs_slow.init.t = obs_slow.setup.time(obs_slow.init.ActualTimeIndex);
    slow_flag = (floor(i/conversion) == (i/conversion)) || (obs_slow.init.t == obs_slow.setup.time(end));
    
    %%%% PROPAGATION %%%%
    % forward propagation of the previous estimate
    for traj = 1:Ntraj
        
        % set trajectories
        obs_fast.init.traj = traj;
        obs_slow.init.traj = traj;
                 
        % propagate only if the time gets over the initial time instant
        if (obs_fast.init.ActualTimeIndex > 1)
            
            % define the time span of the integration
            startpos_fast = obs_fast.init.ActualTimeIndex-1;
            stoppos_fast = obs_fast.init.ActualTimeIndex;
            tspan_fast = obs_fast.setup.time(startpos_fast:stoppos_fast); 
            
            % fast
            X = obs_fast.setup.ode(@(t,x)obs_fast.setup.model(t, x, obs_fast.init.params, obs_fast), tspan_fast, obs_fast.init.X_est(traj).val(:,startpos_fast),params_fast.odeset);
            obs_fast.init.X_est(traj).val(:,startpos_fast:stoppos_fast) = [X.y(:,1),X.y(:,end)];
                                                      
        end
        
        if (obs_slow.init.ActualTimeIndex > 1) && (slow_flag)
            % time span again
            startpos_slow = obs_slow.init.ActualTimeIndex-1;
            stoppos_slow = obs_slow.init.ActualTimeIndex;
            tspan_slow = obs_slow.setup.time(startpos_slow:stoppos_slow);
            
            % slow
            X = obs_slow.setup.ode(@(t,x)obs_slow.setup.model(t, x, obs_slow.init.params, obs_slow), tspan_slow, obs_slow.init.X_est(traj).val(:,startpos_slow),params_slow.odeset);
            obs_slow.init.X_est(traj).val(:,startpos_slow:stoppos_slow) = [X.y(:,1),X.y(:,end)];
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
        updateflag = obs_slow.init.just_optimised && slow;
        if updateflag  
            obs_fast.init.params = obs_fast.setup.params.params_update(obs_fast.init.params,obs_slow.init.X_est(1).val(:,end));
            pos_index_slow = nonzeros(obs_fast.init.Y_space);
            obs_fast.init.X_est.val(obs_fast.setup.update_vars,pos_index_slow(1):pos_index_slow(end)) = obs_slow.init.X_est.val(obs_fast.setup.update_vars,pos_index_slow(1):pos_index_slow(end));
        end        
        tfast = tic;
        obs_fast = obs_fast.observer(obs_fast.init.X_est,y_meas);            
        obs_fast.init.iter_time(obs_fast.init.ActualTimeIndex) = toc(tfast);
    end
    
    % call the observer       
    if slow && slow_flag
        if obs_fast.init.just_optimised && fast && update_slow
            
            % backward integration
            if 1
                
                % initial position
                pos_x0 = find(obs_fast.setup.time == obs_slow.init.t);   
                X0 = obs_fast.init.X_est(traj).val(:,pos_x0);  
                
                % tspan
                pos_index_slow = nonzeros(obs_slow.init.Y_space);                  
                
                if ~isempty(pos_index_slow)
                    time_slow = obs_slow.setup.time(pos_index_slow(1));
                    pos_xstart = obs_fast.init.ActualTimeIndex;
                    pos_xend = find(obs_fast.setup.time == time_slow);
                    tspan_b_pos = fliplr(pos_xend:pos_xstart);
                    tspan_b = obs_fast.setup.time(tspan_b_pos);  
                    
                    % integrate and update
                    X = odeDD(@(t,x)model_battery_tushar_backward(t, x, obs_fast.init.params, obs_fast), tspan_b, X0, params_fast.odeset);

                    % update params
                    Xupdate = X.y(:,1);
                    obs_slow.init.params = obs_slow.setup.params.params_update(obs_slow.init.params,Xupdate);      
                    range_fast = fliplr((tspan_b_pos(end):conversion:tspan_b_pos(1))-tspan_b_pos(end)+1);
                    range_slow = (pos_index_slow:obs_slow.init.ActualTimeIndex);
                    obs_slow.init.X_est(traj).val(:,range_slow) = X.y(:,range_fast);
                end
            end
            
           
            
        end
        
        % observer
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

