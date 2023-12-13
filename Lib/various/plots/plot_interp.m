%% function 
function plot_interp(obs,str,pos,params,fig,sim,range)

    

    % create data
    if ~sim
        data = obs.init.cloud_X(1,:);
        % sort in ascending order
        [data_s, idx_s] = sort(data);
    else
        data_s = obs.init.X_est.val(1,:);
    end
    
    
    % compute poly
    for p=1:length(params)
       data_p(p,:) = data_s.^(p-1); 
    end

    % plot
    figure(fig)
    hold on
    grid on
    box on
    
    if ~sim
        plot(obs.init.params.input_soc,str,'p','LineWidth',2,'MarkerSize',20)  
        plot(data_s,obs.init.cloud_Y(pos,idx_s),'r+','LineWidth',2)
        
        % plot estimation
        plot(data_s,params'*data_p,'m--o','LineWidth',2) 
    else        
        % plot estimation
        plot(data_s,params'*data_p,'b--o','LineWidth',2)            
    end

    


end