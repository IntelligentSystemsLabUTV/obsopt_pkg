%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE out.observer on general model
% INPUT: none
% OUTPUT: params,out.obs
function plot_general(out)

    

    % init section
    i = 0;
    N = length(out.x_dist);
    
    % state plot    
    i = i + 1;            
    figure(i)
    sgtitle('State')    
    for j=1:length(out.obs.setup.plot_vars)
        subplot(length(out.obs.setup.plot_vars),1,j);
        hold on
        grid on
        box on
        plot(out.time,out.x_true.val(out.obs.setup.plot_vars(j),:),'b','LineWidth',1.5);
        for n=1:N
            plot(out.time,out.x_dist(n).val(out.obs.setup.plot_vars(j),:),'r:','LineWidth',1);           
        end
        % labels
        xlabel(['time [s]'])
        ylabel(['x_',num2str(out.obs.setup.plot_vars(j))])
        legend('ref','test')
    end
    
    % output plot    
    i = i + 1;            
    figure(i)
    sgtitle('Measurements')
    % number fo subplots depending on the Nterm
    n_subplot = out.obs.setup.Nfilt+1;
    for k=1:n_subplot
        % indicize axes
        ax_index = k;
        ax(ax_index)=subplot(n_subplot,1,ax_index);
                
        hold on
        grid on
        box on
        
        plot(out.time,reshape(out.y_true.val(k,1,:),1,size(out.y_true.val,3)),'b','LineWidth',1.5);
        for n=1:N
            plot(out.time,reshape(out.y_dist(n).val(k,:,:),1,size(out.y_dist(n).val,3)),'r:','LineWidth',1);           
        end        
%         set(gca, 'YScale', 'log')
        % labels
        xlabel(['time [s]'])
        ylabel('y')
        legend('ref','test')
    end
    
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
