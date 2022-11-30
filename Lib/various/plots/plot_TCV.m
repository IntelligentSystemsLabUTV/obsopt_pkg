%%
% plot results for control design
    function plot_TCV(obj)

        set(0,'DefaultFigureWindowStyle','docked');

        fig_count = 0;

        %%%% plot state estimation %%%
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Ref and Control')
        range = 1:(obj.setup.params.dim_state_r+obj.setup.params.dim_state_c);
        for i=range
            subplot(length(range),1,i-range(1)+1);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_vars(i),:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_vars(i),:),'r--');                                      

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
                else
                    legend('Stored','Est','Runtime')
                end
            end

            % labels
            xlabel(['time [s]'])
            ylabel(['x_',num2str(obj.setup.plot_vars(i))])
        end
        
        %%%% plot state estimation %%%
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Optimizer')
        range = (obj.setup.params.dim_state_r+obj.setup.params.dim_state_c+1):(obj.setup.params.dim_state_r+obj.setup.params.dim_state_c+obj.setup.params.dim_state_op);
        for i=range
            subplot(length(range),1,i-range(1)+1);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_vars(i),:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_vars(i),:),'r--');                                      

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
                else
                    legend('Stored','Est','Runtime')
                end
            end

            % labels
            xlabel(['time [s]'])
            ylabel(['x_',num2str(obj.setup.plot_vars(i))])
        end

        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Annihilator')
        range = (obj.setup.params.dim_state_r+obj.setup.params.dim_state_c+obj.setup.params.dim_state_op+1):(obj.setup.params.dim_state_r+obj.setup.params.dim_state_c+obj.setup.params.dim_state_op+obj.setup.params.dim_state_an);
        for i=range
            subplot(length(range),1,i-range(1)+1);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_vars(i),:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_vars(i),:),'r--');                                      

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
                else
                    legend('Stored','Est','Runtime')
                end
            end

            % labels
            xlabel(['time [s]'])
            ylabel(['x_',num2str(obj.setup.plot_vars(i))])
        end
        
        %%%% plot state estimation %%%
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('State estimation')
        range = (obj.setup.params.dim_state_r+obj.setup.params.dim_state_c+obj.setup.params.dim_state_op+obj.setup.params.dim_state_an+1):(obj.setup.params.dim_state_r+...
                obj.setup.params.dim_state_c+obj.setup.params.dim_state_op+obj.setup.params.dim_state_an+obj.setup.params.n);
        for i=range
            subplot(length(range),1,i-range(1)+1);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_vars(i),:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_vars(i),:),'r--');                                      

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
                else
                    legend('Stored','Est','Runtime')
                end
            end

            % labels
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
                    if strcat(obj.setup.DataType,'simulated')
                        plot(obj.setup.time,obj.init.X(traj).val(obj.setup.plot_params(i),:),'b--');
                    end
                    plot(obj.setup.time,obj.init.X_est(traj).val(obj.setup.plot_params(i),:),'r--');                                      

                    if strcat(obj.setup.DataType,'simulated')
                        legend('True','Est')
                    else
                        legend('Stored','Est','Runtime')
                    end
                end

                % labels
                xlabel(['time [s]'])
                ylabel(['x_',num2str(obj.setup.plot_params(i))])
            end
        end

        %%%% plot state estimation error %%%
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Estimation error - components')

        range = (obj.setup.params.dim_state_r+obj.setup.params.dim_state_c+obj.setup.params.dim_state_op+obj.setup.params.dim_state_an+1):(obj.setup.params.dim_state_r+...
                obj.setup.params.dim_state_c+obj.setup.params.dim_state_op+obj.setup.params.dim_state_an+obj.setup.params.n);
        for i=range
            subplot(length(range),1,i-range(1)+1);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                % plot
                est_error = obj.init.X(traj).val(obj.setup.plot_vars(i),:) - obj.init.X_est(traj).val(obj.setup.plot_vars(i),:);
                
                log_flag = 1;
                if ~log_flag
                    plot(obj.setup.time,est_error,'k','LineWidth',2);
                else
                    set(gca, 'YScale', 'log')
                    plot(obj.setup.time,abs(est_error),'k','LineWidth',2);
                end
            end            

            xlabel('time [s]')
            ylabel(['\delta x_',num2str(obj.setup.plot_vars(i))])
        end

        %%%% plot parameters estimation error %%%
        if ~isempty(obj.setup.plot_params)
            fig_count = fig_count+1;
            figure(fig_count)
            sgtitle('Estimation error - parameters')

            for i=1:length(obj.setup.plot_params)
                subplot(length(obj.setup.plot_params),1,i);
                hold on
                grid on
                box on

                for traj=1:obj.setup.Ntraj
                    % plot
                    est_error = obj.init.X(1).val(obj.setup.plot_params(i),:) - obj.init.X_est(1).val(obj.setup.plot_params(i),:);
                    log_flag = 1;
                    if ~log_flag
                        plot(obj.setup.time,est_error,'k','LineWidth',2);
                    else
                        set(gca, 'YScale', 'log')
                        plot(obj.setup.time,abs(est_error),'k','LineWidth',2);
                    end
                end                

                xlabel('time [s]')
                ylabel(['\delta x_',num2str(obj.setup.plot_params(i))])
            end
        end

        %%%% plot state estimation error - norm%%%
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Estimation error state - norm')
        hold on
        grid on
        box on
        
        est_error_norm = [];
        for traj=1:obj.setup.Ntraj
            % plot
            for iter=1:obj.setup.Niter
                est_error_norm(iter) = norm(obj.init.X(traj).val(obj.setup.plot_vars,iter) - obj.init.X_est(traj).val(obj.setup.plot_vars,iter));            
            end

            log_flag = 1;
            if ~log_flag
                plot(obj.setup.time,est_error_norm,'k','LineWidth',2);
            else
                set(gca, 'YScale', 'log')
                plot(obj.setup.time,abs(est_error_norm),'k--','LineWidth',2);
            end
        end
        
        xlabel('time [s]')
        ylabel('\delta x_norm') 

        %%%% plot params estimation error - norm%%%
        fig_count = fig_count+1;
        figure(fig_count)
        sgtitle('Estimation error params - norm')
        hold on
        grid on
        box on

        est_error_norm = [];
        for traj=1:obj.setup.Ntraj
            
            % plot
            for iter=1:obj.setup.Niter
                est_error_norm(iter) = norm(obj.init.X(traj).val(obj.setup.plot_params,iter) - obj.init.X_est(traj).val(obj.setup.plot_params,iter));                                
            end   
            
            log_flag = 1;
            if ~log_flag
                plot(obj.setup.time,est_error_norm,'k','LineWidth',2);
            else
                set(gca, 'YScale', 'log')
                plot(obj.setup.time,abs(est_error_norm),'k--','LineWidth',2);
            end
        end                

        xlabel('time [s]')
        ylabel('\delta x_norm') 
        
        %%%% plot input difference %%%%
        if ~isempty(obj.init.input_story)
            fig_count = fig_count+1;
            figure(fig_count)
            sgtitle('Allocated input - absolute')

            for i=1:obj.setup.dim_input    
                subplot(obj.setup.dim_input,1,i)
                hold on
                grid on
                box on

                for traj=1:obj.setup.Ntraj
                    % plot
                    u_def = obj.init.input_default_story(traj).val(i,:);                    
                    u_all = obj.init.input_story(traj).val(i,:);
                    plot(obj.setup.time(1:length(u_def)),u_def,'k--','LineWidth',1.5);
                    plot(obj.setup.time(1:length(u_all)),u_all,'r','LineWidth',1.5);

                end                                                
            end
            xlabel('time [s]')
            ylabel('inputs')
            legend('u_{def}','u_{all}')
        end
        
        %%%% plot input difference %%%%
        if ~isempty(obj.init.input_story)
            fig_count = fig_count+1;
            figure(fig_count)
            sgtitle('Allocated input - difference')

            for i=1:obj.setup.dim_input
                subplot(obj.setup.dim_input,1,i);
                hold on
                grid on
                box on

                for traj=1:obj.setup.Ntraj
                    % plot
                    est_error_all = obj.init.input_default_story(traj).val(i,:) - obj.init.input_story(traj).val(i,:);                    
                    log_flag = 0;
                    if ~log_flag
                        plot(obj.setup.time(1:length(est_error_all)),est_error_all,'k','LineWidth',2);                        
                    else
                        set(gca, 'YScale', 'log')
                        plot(obj.setup.time(1:length(est_error_all)),abs(est_error_all),'k','LineWidth',2);                        
                    end
                end                
                
                xlabel('time [s]')
                ylabel(['\delta u_',num2str(i)])
            end
        end
        
        %%%% plot output q difference %%%%
        if ~isempty(obj.init.input_story)
            fig_count = fig_count+1;
            figure(fig_count)
            sgtitle('Reference output - difference')

            for i=1:obj.setup.params.q
                subplot(obj.setup.params.q,1,i);
                hold on
                grid on
                box on

                for traj=1:obj.setup.Ntraj
                    % plot
                    pos = obj.setup.params.q_pos(i);
                    est_error_all = reshape(obj.init.Y_full_story(traj).val(1,pos,:),1,obj.setup.Niter) - reshape(obj.init.Yhat_full_story(traj).val(1,pos,:),1,obj.setup.Niter);
                    plot(obj.setup.time,est_error_all,'r','LineWidth',2);
                end                
                
                xlabel('time [s]')
                ylabel(['\delta yq_',num2str(i)])
            end
        end

        %%%% plot windowed data %%%%
        fig_count = fig_count+1;
        figure(fig_count)        
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
            grid on

            % dpwn sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);

            for traj=1:obj.setup.Ntraj
                % plot true values
                y_meas = reshape(obj.init.Ytrue_full_story(traj).val(1,k,:),size(obj.setup.time));
                plot(obj.setup.time,y_meas,'m--')
                
                % plot ref values
                y_ref = reshape(obj.init.Y_full_story(traj).val(1,k,:),size(obj.setup.time));
                plot(obj.setup.time,y_ref,'k')

                % plot estimated values
                yhat = reshape(obj.init.Yhat_full_story(traj).val(1,k,:),size(obj.setup.time));
                plot(obj.setup.time,yhat,'r--','LineWidth',1.5)


                try
                    data = reshape(obj.init.target_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
                catch 
                    disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
                end

                ylabel(strcat('y_',num2str(k)));
                xlabel('simulation time [s]');
                legend('real','est')
            end
        end
        linkaxes(ax,'x');       

    end