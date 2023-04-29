%% run the observer with no noise 
[obs,params] = simulation_rover;

%% save vars
out.X = obs.init.X;
out.IMU_raw = squeeze(obs.init.Y_full_story.val(1,params.pos_acc_out,1:params.IMU_samp:end)).';
out.UWB_raw = squeeze(obs.init.Y_full_story.val(1,params.pos_dist_out,1:params.UWB_samp:end)).';
out.IMU_resamp = out.IMU_raw;
out.UWB_resamp = out.UWB_raw;
out.IMU = out.IMU_raw;
out.UWB = out.UWB_raw;
out.IMUtime_raw = obs.setup.time(1:params.IMU_samp:end);
out.UWBtime_raw = obs.setup.time(1:params.UWB_samp:end);
out.IMUtime_resamp = out.IMUtime_raw;
out.UWBtime_resamp = out.UWBtime_raw;
out.tscale = params.UWB_samp/params.IMU_samp;
out.IMUt = obs.setup.Ts*params.IMU_samp;
out.UWBt = obs.setup.Ts*params.UWB_samp;
out.F_IMU = NaN;
out.F_UWB = NaN;
out.input_story = obs.init.input_story;