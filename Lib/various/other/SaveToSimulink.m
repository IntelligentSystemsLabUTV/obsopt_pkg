%%
function [SIMstate, SIMinput, SIMmeasure] = SaveToSimulink(obs,runtime)

    % create the bus to be provided to simulink    
    if ~runtime
        SIMstate = timeseries(obs.init.X_est.val,obs.setup.time);
        SIMmeasure = timeseries(reshape(obs.init.Yhat_full_story.val(1,1,:),size(obs.setup.time)),obs.setup.time);
    else
        SIMstate = timeseries(obs.init.X_est_runtime.val,obs.setup.time);
        SIMmeasure = timeseries(reshape(obs.init.Yhat_full_story_runtime.val(1,1,:),size(obs.setup.time)),obs.setup.time);
    end
    SIMinput = timeseries([0 obs.init.input_story.val(1,:)],obs.setup.time);
    

end