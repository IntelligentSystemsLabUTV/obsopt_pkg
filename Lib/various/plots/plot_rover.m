%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE out.observer on general model
% INPUT: none
% OUTPUT: params,out.obs
% plot results for control design
function plot_rover(obj,varargin)
    
    set(0,'DefaultFigureWindowStyle','docked');            
    
    fontsize = 20;
    fig_count = 0;
    
    if length(varargin) == 1
        params = varargin{1};
    end
    
    %%%% plot state estimation %%%
    fig_count = fig_count+1;
    figure(fig_count)            
    sgtitle('State estimation')
    for i=1:length(obj.setup.plot_vars)
        subplot(length(obj.setup.plot_vars),1,i);
        hold on
        grid on
        box on
        
        for traj=1:obj.setup.Ntraj            
            plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_vars(i),:),'--','LineWidth',2);
            set(gca,'ColorOrderIndex',traj)
            plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_vars(i),:),'LineWidth',2);                                                             
            legend('True','Est')            
        end
        
        % labels
        set(gca,'fontsize', fontsize)         
        ylabel(['x_',num2str(obj.setup.plot_vars(i))])
    end
    xlabel(['time [s]'])
    
    %%%% plot parameters estimation %%%
    if ~isempty(obj.setup.plot_params)                
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Parameters estimation')
        for i=1:length(obj.setup.plot_params)
            subplot(length(obj.setup.plot_params),1,i);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_params(i),:),'--','LineWidth',2);                
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_params(i),:),'LineWidth',2);                                                      
            end

            % labels
            set(gca,'fontsize', fontsize)             
            ylabel(['x_',num2str(obj.setup.plot_params(i))])
        end
        xlabel(['time [s]'])
        legend('True','Est')                
    end      
    
    
    %%%% plot windowed data %%%%            
    fig_count = fig_count+1;
    figure(fig_count)
    subplot(2,1,1)    
    sgtitle('Sampled measured')
    ax = zeros(1,3);
    for k=1:length(obj.setup.params.dim_out_plot)
        
        % number fo subplots depending on the output dimension
        n_subplot = length(obj.setup.params.dim_out_plot);
        
        % indicize axes
        ax_index = k;
        ax(ax_index)=subplot(n_subplot,1,ax_index);
        
        % hold on on plots
        hold on
        grid on

        % dpwn sampling instants
        WindowTime = obj.setup.time(obj.init.temp_time);
        
        for traj=1:obj.setup.Ntraj
            % plot true values
            y_meas = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.dim_out_plot(k),:),size(obj.setup.time));
            y_true = reshape(obj.init.Ytrue_full_story(traj).val(1,obj.setup.params.dim_out_plot(k),:),size(obj.setup.time));
            plot(obj.setup.time,y_meas,':','LineWidth',2)
            set(gca,'ColorOrderIndex',traj)
            plot(obj.setup.time,y_true,'LineWidth',2)
            set(gca,'ColorOrderIndex',traj)

            % plot target values    
            try
                data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.dim_out_plot(k),obj.init.temp_time),1,length(WindowTime));
                plot(WindowTime,data,'o','MarkerSize',5);
            catch 
                disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
            end

            set(gca,'fontsize', fontsize)
            ylabel(strcat('y_',num2str(k)));            
        end
    end
    xlabel('simulation time [s]');
    legend('meas','true','sampled')
    linkaxes(ax(1:n_subplot-1),'x');

    %%% rover trajectory
    fig_count = fig_count+1;
    figure(fig_count)
    hold on
    grid on
    % plot anchors
    P_a(1,:) = obj.init.params.pos_anchor(1):3:obj.init.params.pos_anchor(end);
    P_a(2,:) = obj.init.params.pos_anchor(2):3:obj.init.params.pos_anchor(end); 
    P_a(3,:) = obj.init.params.pos_anchor(3):3:obj.init.params.pos_anchor(end); 
    for i=1:obj.setup.params.Nanchor
        plot3(obj.init.X_est(1).val(P_a(1,i),:),obj.init.X_est(1).val(P_a(2,i),:),obj.init.X_est(1).val(P_a(3,i),:),'ko','MarkerSize',10);
    end
    % plot rover
    pos_p = obj.init.params.pos_p;
    for traj=1:obj.setup.Ntraj 
        plot3(obj.init.X_est(traj).val(pos_p(1),:),obj.init.X_est(traj).val(pos_p(2),:),obj.init.X_est(traj).val(pos_p(3),:),'--');
        set(gca,'ColorOrderIndex',traj)
        plot3(obj.init.X(traj).val(pos_p(1),:),obj.init.X(traj).val(pos_p(2),:),obj.init.X_est(traj).val(pos_p(3),:));    
    end
    xlabel('X')
    ylabel('Y')
    set(gca,'fontsize', fontsize)

    %%% check distances %%%
    fig_count = fig_count+1;
    figure(fig_count)    
    for n=1:obj.init.params.Nanchor
        ax(n) = subplot(obj.init.params.Nanchor,1,n);
        hold on
        grid on  
        for traj=1:obj.setup.Ntraj
            plot(obj.setup.time(obj.init.params.UWB_pos),squeeze(obj.init.Y_full_story(traj).val(1,obj.init.params.pos_dist_out(n),obj.init.params.UWB_pos)),'LineWidth',2);
            set(gca,'ColorOrderIndex',traj)
            plot(obj.setup.time(obj.init.params.UWB_pos),squeeze(obj.init.Yhat_full_story(traj).val(1,obj.init.params.pos_dist_out(n),obj.init.params.UWB_pos)),':','LineWidth',2);
            set(gca,'ColorOrderIndex',traj)
            plot(obj.setup.time(obj.init.params.UWB_pos),squeeze(obj.init.Ytrue_full_story(traj).val(1,obj.init.params.pos_dist_out(n),obj.init.params.UWB_pos)),'--','LineWidth',2);
        end        
        ylabel(['d_',num2str(n)])        
        set(gca,'fontsize', fontsize)
    end   
    xlabel('time [s]')
    legend('meas','opt','true');            
    
end