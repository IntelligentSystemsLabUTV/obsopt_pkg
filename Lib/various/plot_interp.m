%% function 
function plot_interp(obs,str,pos,params,wholeSim,fig)

    

    % create data
    data = obs.init.cloud_X(1,:);
%     data = obs.init.X_est(1).val(1,:);
    
    % sort in ascending order
    [data_s, idx_s] = sort(data);
    
    % compute poly
    for p=1:length(params)
       data_p(p,:) = data_s.^(p-1); 
    end

    % plot
    figure(fig)
    plot(obs.init.params.input_soc,str,'p:','LineWidth',2)
    hold on
    
    if ~wholeSim
        i=1;        
        plot(data_s,obs.init.cloud_Y(pos,idx_s),'r+','LineWidth',2)
%         plot(data_s,spline(obs.init.params.input_soc, str, obs.init.cloud(1,idx_s)),'b:o','LineWidth',2)
%         plot(obs.init.X(1).val(1,i),spline(obs.init.params.input_soc, str, obs.init.X(1).val(1,i)),'k+')   

        % plot estimation
        plot(data_s,params*data_p,'k--o','LineWidth',2)
    else
%         plot(obs.init.X(1).val(1,:),spline(obs.init.params.input_soc, str, obs.init.X(1).val(1,:)),'m--o','LineWidth',2)    
        
        % plot estimation
        plot(data_s,params*data_p,'m--o','LineWidth',2)
    end
    
    
    
    % manipulate
    grid on
    xlabel('Z')
    ylabel('param')
%     legend('experimental data')
    legend('measured','spline interp','linear interp')

end