% plot results for control design
    function out = plot_manipulator(obj,varargin)

        %%%% plot state estimation %%%
        figure(1)
        for i=1:obj.setup.plot_vars
            subplot(obj.setup.plot_vars,1,i);
            hold on
            grid on
            box on

            for traj=1:obj.setup.Ntraj
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(i,:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(i,:),'r.');
%                     plot(obj.setup.time,obj.init.X_est_runtime(traj).val(i,:),'g');

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
%                         legend('Stored','Est','Runtime')
                else
                    legend('Stored','Est','Runtime')
                end
            end
        end

        %%%% plot state estimation error %%%
        figure(2)
        for i=1:obj.setup.plot_vars
            subplot(obj.setup.plot_vars,1,i);
            hold on
            grid on
            box on

            % plot
            est_error = obj.init.X(1).val(i,:) - obj.init.X_est_runtime(1).val(i,:);

            log_flag = 1;
            if ~log_flag
                plot(obj.setup.time,est_error,'k','LineWidth',2);
            else
                % log 
%                     set(gca, 'XScale', 'log')
                set(gca, 'YScale', 'log')
                plot(obj.setup.time,abs(est_error),'k','LineWidth',2);
            end

            xlabel('time [s]')
            ylabel(['\delta x_',num2str(i)])
        end

        %%%% plot state estimation error - norm%%%
        figure(3)
        hold on
        grid on
        box on
        % plot
        for iter=1:obj.setup.Niter
            est_error_norm(iter) = norm(obj.init.X(1).val(1:obj.init.params.dim_state,iter) - obj.init.X_est_runtime(1).val(1:obj.init.params.dim_state,iter));
        end
        log_flag = 1;
        if ~log_flag
            plot(obj.setup.time,est_error_norm,'k','LineWidth',2);
        else
            % log 
%                     set(gca, 'XScale', 'log')
            set(gca, 'YScale', 'log')
            plot(obj.setup.time,abs(est_error_norm),'k-.','LineWidth',2);
        end
        xlabel('time [s]')
        ylabel('\delta x_norm')            


        %%%% plot windowed data %%%%
        figure(4)
        title('Sampled measured')
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
                plot(obj.setup.time,y_meas,'m--')

                % plot down sampling
                data = reshape(obj.init.Yhat_full_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
%                     plot(WindowTime,data,'s','MarkerSize',5);

                % plot target values
                data = reshape(obj.init.target_story(traj).val(1,k,obj.init.temp_time),1,length(WindowTime));
                plot(WindowTime,data,'bo','MarkerSize',5);

                ylabel(strcat('y_',num2str(k)));
                xlabel('simulation time [s]');
%                     legend('true','estimation','target')
                legend('trajectory','measures')
            end
        end
        linkaxes(ax,'x');

        %%%% plot filters %%%%%
        figure(5)
        title('Filters on measures')
        ax = zeros(1,3);
        for k=1:obj.setup.J_nterm

            % number fo subplots depending on the Nterm
            n_subplot = obj.setup.J_nterm;

            % indicize axes
            ax_index = k;
            ax(ax_index)=subplot(n_subplot,1,ax_index);

            % plot
            hold on
            for traj=1:obj.setup.Ntraj
                for dim=1:obj.setup.dim_out
                    y_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Y_full_story(traj).val(k,dim,:),size(obj.setup.time));
                    yhat_plot = obj.setup.J_temp_scale(k)*reshape(obj.init.Yhat_full_story(traj).val(k,dim,:),size(obj.setup.time));
                    plot(obj.setup.time,y_plot,'b--');
                    plot(obj.setup.time,yhat_plot,'r.');
                end

                ylabel(strcat('y_filter',num2str(k)));
                xlabel('simulation time [s]');
            end            

        end
        linkaxes(ax,'x');
        
        % plot workspace
        figure(6)
        hold on
        grid on
        box on
        title('Manipulator workspace')
        for traj=1:obj.setup.Ntraj
            % target
            [~, yall] = measure_manipulator(obj.init.X(traj).val(:,1),obj.init.params);
            plot(yall(1,:),yall(2,:),'b+-','LineWidth',2);
            out.dist_true(1,1) = norm(yall(:,2)-yall(:,1));
            out.dist_true(2,1) = norm(yall(:,3)-yall(:,2));
             
            % controlled
            for i=1:size(obj.init.Yhat_full_story(traj).val,3)
                [~, yall] = measure_manipulator(obj.init.X_est(traj).val(:,i),obj.init.params);
                plot(yall(1,:),yall(2,:),'k-.','LineWidth',0.1); 
                out.dist_hat(1,i) = norm(yall(:,2)-yall(:,1));
                out.dist_hat(2,i) = norm(yall(:,3)-yall(:,2));
            end            
             
            % start - controlled 
            [~, yall] = measure_manipulator(obj.init.X_est(traj).val(:,1),obj.init.params);
            plot(yall(1,:),yall(2,:),'ro-','LineWidth',2);
             
            % arrival - controlled 
            [~, yall] = measure_manipulator(obj.init.X_est(traj).val(:,end),obj.init.params);
            plot(yall(1,:),yall(2,:),'gs-','LineWidth',2);
        end
        
        %%%% plot state estimation %%%
        figure(7)
        for i=1:obj.setup.dim_state-obj.setup.plot_vars
            subplot(obj.setup.dim_state-obj.setup.plot_vars,1,i);
            hold on
            grid on
            box on
            
            start = obj.setup.plot_vars + i;

            for traj=1:obj.setup.Ntraj
                if strcat(obj.setup.DataType,'simulated')
                    plot(obj.setup.time,obj.init.X(traj).val(start,:),'b--');
                end
                plot(obj.setup.time,obj.init.X_est(traj).val(start,:),'r.');
%                     plot(obj.setup.time,obj.init.X_est_runtime(traj).val(i,:),'g');

                if strcat(obj.setup.DataType,'simulated')
                    legend('True','Est')
%                         legend('Stored','Est','Runtime')
                else
                    legend('Stored','Est','Runtime')
                end
            end
        end

    end