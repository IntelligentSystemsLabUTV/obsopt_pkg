%% function 
function [params_s, obs_s, time, SimParams, params, SIMstate, SIMinput, SIMmeasure] = InitMultiMHE(tstart,tend,Ts)

    Ts_fast = Ts(1);
    Ts_slow = Ts(2);
    
    % MHE fast
    % set opt vars
    system("sed -i 's/params.opt_vars =.*/params.opt_vars = [1:2];/' Lib/models/battery/params_battery_tushar.m");
    pause(1);
    system("sed -i 's/fil1 = .*/fil1 = 0;/' Lib/measure/filter_define.m");
    pause(1);
    system("sed -i 's/fil2 = .*/fil2 = 0;/' Lib/measure/filter_define.m");
    pause(1);
    [obs_fast, params_fast, ~] = setup_model_fast(tstart,tend,Ts_fast);

    % MHE slow
    % set opt vars
    % 12:14 16:18 20:22
    system("sed -i 's/params.opt_vars =.*/params.opt_vars = [1:2 8:10 12:14 16:18 20:22];/' Lib/models/battery/params_battery_tushar.m");
    pause(1);
    system("sed -i 's/fil1 = .*/fil1 = 0;/' Lib/measure/filter_define.m");
    pause(1);
    system("sed -i 's/fil2 = .*/fil2 = 0;/' Lib/measure/filter_define.m");
    pause(1);
    system("sed -i 's/fil3 = .*/fil3 = 0;/' Lib/measure/filter_define.m");
    pause(1);
    [obs_slow, params_slow, ~] = setup_model_slow(tstart,tend,Ts_slow);
    
    %%%% first guess %%%%
    Ts_slower = min(Ts);
    first_guess_flag = 1;
    [obs, ~, ~] = setup_model_fast(tstart,tend,Ts_slower);
    if first_guess_flag
        first_guess;   
        % noise on parameters - is it really necessary?
        params_fast.X0 = obs.init.X_est.val(:,1);
        params_slow.X0 = obs.init.X_est.val(:,1);
        pert = (1+0.0*ones(obs.setup.dim_state,1));
        obs.init.X_est.val(:,1) = obs.init.X_est.val(:,1).*pert;

        % update objects        
        obs_fast.init.X_est.val(:,1) = obs.init.X_est.val(:,1);
        obs_slow.init.X_est.val(:,1) = obs.init.X_est.val(:,1);
        obs_fast.init.params = obs_fast.setup.params.params_update(obs_fast.init.params,obs_fast.init.X_est(1).val(:,1));
        obs_slow.init.params = obs_slow.setup.params.params_update(obs_slow.init.params,obs_slow.init.X_est(1).val(:,1));
        obs_fast.setup.params = obs_fast.setup.params.params_update(obs_fast.setup.params,obs_fast.init.X_est(1).val(:,1));
        obs_slow.setup.params = obs_slow.setup.params.params_update(obs_slow.setup.params,obs_slow.init.X_est(1).val(:,1));
        obs_fast.init.cloud_X = obs.init.cloud_X;
        obs_slow.init.cloud_X = obs.init.cloud_X;
        obs_fast.init.cloud_Y = obs.init.cloud_Y;
        obs_slow.init.cloud_Y = obs.init.cloud_Y;                
    end
    clear obs;        
    
    % set stuff
    time = params_fast.time;
    params_s = {params_fast, params_slow};
    obs_s = {obs_fast, obs_slow};
    
    % set simulink
    [obs_tmp, ~, SimParams] = setup_model;
    params = params_s{1};
    params.Ts_slower = Ts_slower;
    noise = 1;
    params.percNoise = noise*5e-2;
    params.NoisePwr = noise*5e-2;
    
    obs_tmp.init.X_est.val = zeros(size(obs_tmp.init.X_est.val(:,1),1),length(obs_tmp.setup.time));
    obs_tmp.init.input_story.val = zeros(obs_tmp.setup.params.dim_input,length(obs_tmp.setup.time)-1);
    obs_tmp.init.Yhat_full_story.val = zeros(size(obs_tmp.init.X_est.val(:,1),1),1,length(obs_tmp.setup.time));
    [SIMstate, SIMinput, SIMmeasure] = SaveToSimulink(obs_tmp,0);

end