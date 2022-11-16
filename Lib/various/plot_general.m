%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE out.observer on general model
% INPUT: none
% OUTPUT: params,out.obs
% plot results for control design
function plot_general(obj,varargin)
    
    set(0,'DefaultFigureWindowStyle','docked');            
    
    fontsize = 20;
    fig_count = 0;
    
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
            if 1 || strcmp(obj.setup.DataType,'simulated')
                plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_vars(i),:),'b--','LineWidth',2);
            end
            plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_vars(i),:),'r--','LineWidth',2);                                      

            if strcat(obj.setup.DataType,'simulated')
                legend('True','Est')
            else
                legend('Stored','Est','Runtime')
            end
        end
        
        % labels
        set(gca,'fontsize', fontsize) 
        xlabel(['time [s]'])
        ylabel(['x_',num2str(obj.setup.plot_vars(i))])
    end
    
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
                if 1 || strcmp(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_params(i),:),'b--','LineWidth',2);
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_params(i),:),'g--','LineWidth',2);                                      

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
                else
                    legend('Stored','Est','Runtime')
                end
            end

            % labels
            set(gca,'fontsize', fontsize) 
            xlabel(['time [s]'])
            ylabel(['x_',num2str(obj.setup.plot_params(i))])
        end
    end
    
    %%%% plot state estimation error %%%
    if strcmp(obj.setup.DataType,'simulated')                
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Estimation error - components')

        for i=1:length(obj.setup.plot_vars)
            subplot(length(obj.setup.plot_vars),1,i);
            hold on
            grid on
            box on

            % plot
            est_error = obj.init.X(1).val(obj.setup.plot_vars(i),:) - obj.init.X_est(1).val(obj.setup.plot_vars(i),:);

            log_flag = 1;
            if ~log_flag
                plot(obj.setup.time,est_error,'k','LineWidth',2);
            else
                % log 
%                     set(gca, 'XScale', 'log')
                set(gca, 'YScale', 'log')
                plot(obj.setup.time,abs(est_error),'k','LineWidth',2);
            end

            set(gca,'fontsize', fontsize)
            xlabel('time [s]')
            ylabel(['\delta x_',num2str(obj.setup.plot_vars(i))])
        end
    end
    
    %%%% plot parameters estimation error %%%
    if 1 || strcmp(obj.setup.DataType,'simulated')
        if ~isempty(obj.setup.plot_params)                    
            fig_count = fig_count+1;
            figure(fig_count)
            sgtitle('Estimation error - parameters')

            for i=1:length(obj.setup.plot_params)
                subplot(length(obj.setup.plot_params),1,i);
                hold on
                grid on
                box on

                % plot
                est_error = obj.init.X(1).val(obj.setup.plot_params(i),:) - obj.init.X_est(1).val(obj.setup.plot_params(i),:);

                log_flag = 1;
                if ~log_flag
                    plot(obj.setup.time,est_error,'b','LineWidth',2);
                else
                    % log 
%                     set(gca, 'XScale', 'log')
                    set(gca, 'YScale', 'log')
                    plot(obj.setup.time,abs(est_error),'b','LineWidth',2);
                end

                set(gca,'fontsize', fontsize)
                xlabel('time [s]')
                ylabel(['\delta x_',num2str(obj.setup.plot_params(i))])
            end
        end
    end
    
    %%%% plot state estimation error - norm%%%
    if strcmp(obj.setup.DataType,'simulated')                
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Estimation error state - norm')
        hold on
        grid on
        box on

        % plot
        for iter=1:obj.setup.Niter
            est_error_norm(iter) = norm(obj.init.X(1).val(obj.setup.plot_vars,iter) - obj.init.X_est(1).val(obj.setup.plot_vars,iter));
        end

        log_flag = 0;
        if ~log_flag
            plot(obj.setup.time,est_error_norm,'k','LineWidth',2);
        else
            % log 
%                     set(gca, 'XScale', 'log')
            set(gca, 'YScale', 'log')
            plot(obj.setup.time,abs(est_error_norm),'r--','LineWidth',2);
        end

        set(gca,'fontsize', fontsize)
        xlabel('time [s]')
        ylabel('\delta x_norm') 
    end
    
    %%%% plot params estimation error - norm%%%
    if 1 || strcmp(obj.setup.DataType,'simulated')                
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Estimation error params - norm')
        hold on
        grid on
        box on

        % plot
        for iter=1:obj.setup.Niter
            est_error_norm(iter) = norm(obj.init.X(1).val(obj.setup.plot_params,iter) - obj.init.X_est(1).val(obj.setup.plot_params,iter));
        end

        log_flag = 0;
        if ~log_flag
            plot(obj.setup.time,est_error_norm,'r','LineWidth',2);
        else
            % log 
%                     set(gca, 'XScale', 'log')
            set(gca, 'YScale', 'log')
            plot(obj.setup.time,abs(est_error_norm),'b--','LineWidth',2);
        end

        set(gca,'fontsize', fontsize)
        xlabel('time [s]')
        ylabel('\delta x_norm') 
    end
    
    
    %%%% plot windowed data %%%%            
    fig_count = fig_count+1;
    figure(fig_count)
    grid on
    sgtitle('Sampled measured')
    ax = zeros(1,3);
    for k=1:obj.setup.dim_out
        
        % number fo subplots depending on the output dimension
        n_subplot = obj.setup.dim_out;
        
        % indicize axes
        ax_index = k;
        ax(ax_index)=subplot(n_subplot,1,ax_index);
        
        % hold on on plots
        hold on
        
        % dpwn sampling instants
        WindowTime = obj.setup.time(obj.init.temp_time);
        
        for traj=1:obj.setup.Ntraj
            % plot true values
            y_meas = reshape(obj.init.Y_full_story(traj).val(1,k,:),size(obj.setup.time));
            plot(obj.setup.time,y_meas,'m:','LineWidth',2)
            
            % plot estimated values
            yhat = reshape(obj.init.Yhat_full_story(traj).val(1,k,:),size(obj.setup.time));
            plot(obj.setup.time,yhat,'b--','LineWidth',2)
            
            if strcmp(obj.setup.DataType,'simulated')
                y_true = reshape(obj.init.Ytrue_full_story(traj).val(1,k,:),size(obj.setup.time));
                plot(obj.setup.time,y_true,'k--','LineWidth',2)
            end                    

            % plot target values    
            try
                data = reshape(obj.init.target_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
                plot(WindowTime,data,'bo','MarkerSize',5);
            catch 
                disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
            end

            set(gca,'fontsize', fontsize)
            ylabel(strcat('y_',num2str(k)));
            xlabel('simulation time [s]');
            if strcmp(obj.setup.DataType,'simulated')
                legend('meas','estimation','target','sampled')
            else
                legend('meas','est','sampled')
            end
        end
    end
    linkaxes(ax,'x');
    
    %%%% plot filters %%%%%            
    fig_count = fig_count+1;
    figure(fig_count)
    sgtitle('Filters on measures')            
    ax = zeros(1,3);
    for k=1:obj.setup.J_nterm
        
        % number fo subplots depending on the Nterm
        n_subplot = obj.setup.J_nterm;
        
        % indicize axes
        ax_index = k;
        ax(ax_index)=subplot(n_subplot,1,ax_index);                
        
        % plot
        hold on
        grid on
        
        for traj=1:obj.setup.Ntraj
            for dim=1:obj.setup.dim_out
                y_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Y_full_story(traj).val(k,dim,:),size(obj.setup.time));
                if strcmp(obj.setup.DataType,'simulated')
                    ytrue_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Ytrue_full_story(traj).val(k,dim,:),size(obj.setup.time));
                end
                yhat_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Yhat_full_story(traj).val(k,dim,:),size(obj.setup.time));
                if 1
                    if strcmp(obj.setup.DataType,'simulated')
                        plot(obj.setup.time,y_plot,'b--');
                    end
                    plot(obj.setup.time,yhat_plot,'r--','LineWidth',2);
                    plot(obj.setup.time,y_plot,'k:','LineWidth',2);                            
                else
                    plot(obj.setup.time,abs(y_plot-yhat_plot));
                    set(gca, 'YScale', 'log')
                end
            end
            
            if strcmp(obj.setup.DataType,'simulated')
                legend('meas','estimation','target')
            else
                legend('meas','est')
            end
            
            set(gca,'fontsize', fontsize)
            ylabel(strcat('y_{filter}^',num2str(k)));
            xlabel('simulation time [s]');
        end            
        
    end
    linkaxes(ax,'x');
    
    %%% plot adaptive sampling            
    fig_count = fig_count+1;
    figure(fig_count)
    hold on
    grid on
    plot(obj.setup.time,obj.init.dJ_cond_story,'b','LineWidth',1.5)            
    plot(obj.setup.time,ones(obj.setup.Niter,1)*obj.setup.dJ_low,'k--','LineWidth',2)
    plot(obj.setup.time,ones(obj.setup.Niter,1)*obj.setup.dJ_high,'k--','LineWidth',2)
%             set(gca, 'YScale', 'log')            
    plot(obj.setup.time,ones(obj.setup.Niter,1)*obj.setup.dPE,'k:','LineWidth',2)
    plot(obj.setup.time,obj.init.PE(1).val,'r','LineWidth',1.5)
    WindowTime = obj.setup.time(obj.init.temp_time);
    data = obj.init.dJ_cond_story(obj.init.temp_time);
    plot(WindowTime,data,'mo','MarkerSize',5);
    WindowTime = obj.setup.time(obj.init.temp_time);
    data = obj.init.PE(1).val(obj.init.temp_time);
    plot(WindowTime,data,'mo','MarkerSize',5);
    set(gca,'fontsize', fontsize)
    xlabel('time [s]');
    ylabel('adaptive condition');
    
end