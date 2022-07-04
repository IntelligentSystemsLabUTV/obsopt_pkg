%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE observer on general model
% INPUT: none
% OUTPUT: params,obs
function plot_general(out,obs)

    % init section
    i = 0;
    N = length(out.x_dist);
    
    % state plot    
    i = i + 1;            
    figure(i)
    sgtitle('State')    
    for j=1:length(obs.setup.plot_vars)
        subplot(length(obs.setup.plot_vars),1,j);
        hold on
        grid on
        box on
        plot(out.time,out.x_true.val(obs.setup.plot_vars(j),:),'b');
        for n=1:N
            plot(out.time,out.x_dist(n).val(obs.setup.plot_vars(j),:),'r--');           
        end
        % labels
        xlabel(['time [s]'])
        ylabel(['x_',num2str(obs.setup.plot_vars(j))])
        legend('ref','test')
    end
    
    % output plot    
    i = i + 1;            
    figure(i)
    sgtitle('Output')        
    hold on
    grid on
    box on
    plot(out.time,reshape(out.y_true.val(1,1,:),1,size(out.y_true.val,3)),'b');
    for n=1:N
        plot(out.time,reshape(out.y_dist(n).val(1,:,:),1,size(out.y_dist(n).val,3)),'r--');           
    end
    % labels
    xlabel(['time [s]'])
    ylabel('y')
    legend('ref','test')
    
    % error plot    
    i = i + 1;            
    figure(i)
    sgtitle('Output - error')        
    hold on
    grid on
    box on
    for n=1:N
        plot(out.time,abs(reshape(out.y_true.val(1,:,:)-out.y_dist(n).val(1,:,:),1,size(out.y_dist(n).val,3))),'r--');           
    end
    set(gca, 'YScale', 'log')
    % labels
    xlabel(['time [s]'])
    ylabel('y')
    legend('error')  

end
