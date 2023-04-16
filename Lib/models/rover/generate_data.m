%% run the observer with no noise 
[obs,params] = simulation_rover;

%% save vars
out.X = obs.init.X;
out.IMU_raw = reshape(obs.init.Ytrue_full_story.val(1,params.pos_acc_out,:),obs.setup.Niter,params.space_dim);
out.UWB_raw = reshape(obs.init.Ytrue_full_story.val(1,params.pos_dist_out,:),obs.setup.Niter,params.Nanchor);
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