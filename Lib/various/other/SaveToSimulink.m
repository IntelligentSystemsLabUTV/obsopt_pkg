%%
function [SIMstate, SIMinput, SIMmeasure] = SaveToSimulink(obs,time,runtime)

    % create the bus to be provided to simulink    
    if ~runtime
        SIMstate = timeseries(obs.init.X_est.val,time);
        SIMmeasure = timeseries(reshape(obs.init.Yhat_full_story.val(1,1,:),size(time)),time);
    else
        SIMstate = timeseries(obs.init.X_est_runtime.val,time);
        SIMmeasure = timeseries(reshape(obs.init.Yhat_full_story_runtime.val(1,1,:),size(time)),time);
    end
    SIMinput = timeseries([0 obs.init.input_story.val(1,:)],time);
    

end