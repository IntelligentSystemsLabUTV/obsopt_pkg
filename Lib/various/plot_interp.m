%% function 
function plot_interp(obs,str,pos,params,wholeSim)

    % create data
    if ~wholeSim
        data = obs.init.cloud(1,:);
    else
        data = obs.init.X_est(1).val(1,:);
    end
    
    % compute poly
    for p=1:length(params)
       data_p(p,:) = data.^(p-1); 
    end

    % plot
    figure()
    plot(obs.init.params.input_soc,str)
    hold on
    
    if ~wholeSim
        i=1;
        plot(obs.init.cloud(1,:),spline(obs.init.params.input_soc, str, obs.init.cloud(1,:)),'ro')
        plot(obs.init.cloud(1,:),obs.init.cloud(pos,:),'r+')
        plot(obs.init.X(1).val(1,i),spline(obs.init.params.input_soc, str, obs.init.X(1).val(1,i)),'ko')                        
    else
        plot(obs.init.X(1).val(1,:),spline(obs.init.params.input_soc, str, obs.init.X(1).val(1,:)),'ro')        
    end
    
    % plot estimation
    plot(data,params*data_p,'k+')

end