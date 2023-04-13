%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE out.observer on general model
% INPUT: none
% OUTPUT: params,out.obs
% plot results for control design
function plot_3Dobject_bearing(obj,varargin)
    
    set(0,'DefaultFigureWindowStyle','docked');            
    
    fontsize = 20;
    fig_count = 0;
    
    if length(varargin) == 1
        params = varargin{1};
    else
        params = obj.init.params;
    end
    
    %%%% plot position estimation %%%
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Position estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_p)
            subplot(length(params.pos_p),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_p),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_p(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est(traj).val(params.pos_p(i),:),'--','LineWidth',1);                                                                                                  
            end

            % test
            if i==3
                plot(obj.setup.time(2:end),obj.init.reference_story(1).val(1,:),'k','LineWidth',1.5);
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['p_',num2str(obj.setup.plot_vars(i))])
        end
        %linkaxes(ax);
        legend('True','Est')   
        xlabel(['time [s]']) 
    catch ME
        close
        fig_count = fig_count -1;
    end
    
    %%%% plot bearing estimation %%%%
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Bearing estimation')
        ax = zeros(1,numel(params.pos_theta));
        for i=1:length(params.pos_theta)
            subplot(length(params.pos_theta),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_theta),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_theta(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est(traj).val(params.pos_theta(i),:),'--','LineWidth',1);                                                                                                  
            end            
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['\theta_',num2str(obj.setup.plot_vars(i))])
        end
        %linkaxes(ax);
        legend('True','Est')   
        xlabel(['time [s]']) 
    catch ME
        close
        fig_count = fig_count -1;
    end
    

    %%% plot velocity estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Velocity estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_v)
            subplot(length(params.pos_v),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_v),1,i);     
            
            for traj=1:obj.setup.Ntraj           
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_v_out(i),:)),'LineWidth',2);                
                set(gca,'ColorOrderIndex',traj)
%                 plot(obj.setup.time,squeeze(obj.init.Yhat_full_story(traj).val(1,params.pos_v_out(i),:)),'--','LineWidth',1);                                                                             
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['v_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est') 
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end

    %%% plot acceleration estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Acceleration estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_acc)
            subplot(length(params.pos_acc),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_acc),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_acc_out(i),:)),'LineWidth',2);                
                set(gca,'ColorOrderIndex',traj)
%                 plot(obj.setup.time,squeeze(obj.init.Yhat_full_story(traj).val(1,params.pos_acc_out(i),:)),'--','LineWidth',1);                                                                             
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['a_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est')  
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end

    %%% plot bias estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Bias estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_bias)
            subplot(length(params.pos_bias),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_bias),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_bias(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est(traj).val(params.pos_bias(i),:),'--','LineWidth',1);                                                                                   
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['b_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est') 
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end
    
    
    %%%% plot pos measure data %%%%   
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('pos measure')
        ax = zeros(1,3);
        for i=1:length(params.pos_p_out)
            subplot(length(params.pos_p_out),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_p_out),1,i);    

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_p_out(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_p_out(i),:)),'--','LineWidth',1);          

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_p_out(i),obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'o','MarkerSize',5);
                catch 
                    disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
                end
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['p_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Meas')            
        xlabel(['time [s]'])   
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end    

    %%%% plot acc measure data %%%%  
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('acc measure')
        ax = zeros(1,3);
        for i=1:length(params.pos_acc_out)
            subplot(length(params.pos_acc_out),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_acc_out),1,i);   

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_acc_out(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_acc_out(i),:)),'--','LineWidth',1);    

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_acc_out(i),obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'o','MarkerSize',5);
                catch 
                    disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
                end
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['a_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Meas')            
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end  
    
    %%%% plot bearing measure data %%%%  
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('bearing measure')
        ax = zeros(1,3);
        for i=1:length(params.pos_theta_out)
            subplot(length(params.pos_theta_out),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_theta_out),1,i);   

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_theta_out(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_theta_out(i),:)),'--','LineWidth',1);    

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_theta_out(i),obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'o','MarkerSize',5);
                catch 
                    disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
                end
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['a_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Meas')            
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end    
    
    try
        %%% check distances %%%
        for tag = 1:obj.init.params.Ntag
            fig_count = fig_count+1;
            figure(fig_count)    
            for n=1:numel(obj.init.params.TagAnchorIndex(tag,:))
                ax(n) = subplot(numel(obj.init.params.TagAnchorIndex(tag,:)),1,n);
                hold on
                grid on  
                for traj=1:obj.setup.Ntraj
                    plot(obj.setup.time(obj.init.params.UWB_pos),squeeze(obj.init.Y_full_story(traj).val(1,obj.init.params.pos_dist_out(obj.init.params.TagAnchorIndex(tag,n)),obj.init.params.UWB_pos)),'LineWidth',2);
                    set(gca,'ColorOrderIndex',traj)
%                     plot(obj.setup.time(obj.init.params.UWB_pos),squeeze(obj.init.Yhat_full_story(traj).val(1,obj.init.params.pos_dist_out(obj.init.params.TagAnchorIndex(tag,n)),obj.init.params.UWB_pos)),':','LineWidth',2);
%                     set(gca,'ColorOrderIndex',traj)
                    plot(obj.setup.time(obj.init.params.UWB_pos),squeeze(obj.init.Ytrue_full_story(traj).val(1,obj.init.params.pos_dist_out(obj.init.params.TagAnchorIndex(tag,n)),obj.init.params.UWB_pos)),'--','LineWidth',2);
                end        
                ylabel(['d_',num2str(n)])        
                set(gca,'fontsize', fontsize)
            end   
            xlabel('time [s]')
            legend('meas','opt','true');
        end
    catch
        close
        fig_count = fig_count -1;
    end

    try
        %%% rover trajectory
        fig_count = fig_count+1;
        figure(fig_count)
        hold on
        grid on
        
        % plot hills
        pos_p = obj.init.params.pos_p;
        pos_theta = obj.init.params.pos_theta;        
        
        % plot anchors
        P_a(1,:) = obj.init.params.pos_anchor(1):3:obj.init.params.pos_anchor(end);
        P_a(2,:) = obj.init.params.pos_anchor(2):3:obj.init.params.pos_anchor(end); 
        P_a(3,:) = obj.init.params.pos_anchor(3):3:obj.init.params.pos_anchor(end); 
        for i=1:obj.setup.params.Nanchor
            if i<= 4
                plot3(obj.init.X_est(1).val(P_a(1,i),:),obj.init.X_est(1).val(P_a(2,i),:),obj.init.X_est(1).val(P_a(3,i),:),'ko','MarkerSize',10,'LineWidth',2);
            else
                plot3(obj.init.X_est(1).val(P_a(1,i),:),obj.init.X_est(1).val(P_a(2,i),:),obj.init.X_est(1).val(P_a(3,i),:),'ks','MarkerSize',10,'LineWidth',2);
            end
        end
        
        % ref traj
        if params.shape == 1
            % ellipse
            plot3(params.aell.*cos(params.K*obj.setup.time),params.bell.*sin(params.K*obj.setup.time),0*obj.setup.time,'k:');
            plot3(params.aell.*cos(params.K*obj.setup.time(1)),params.bell.*sin(params.K*obj.setup.time(1)),0*obj.setup.time(1),'md','MarkerSize',5,'LineWidth',2);
        else
            % rectangle
            plot3(params.aell*(abs(cos(params.K*obj.setup.time)).*cos(params.K*obj.setup.time) + abs(sin(params.K*obj.setup.time)).*sin(params.K*obj.setup.time)), ...
                params.bell*(abs(cos(params.K*obj.setup.time)).*cos(params.K*obj.setup.time) - abs(sin(params.K*obj.setup.time)).*sin(params.K*obj.setup.time)), ...
                0*obj.setup.time,'r--');            
            plot3(params.aell*(abs(cos(params.K*obj.setup.time(1))).*cos(params.K*obj.setup.time(1)) + abs(sin(params.K*obj.setup.time(1))).*sin(params.K*obj.setup.time(1))), ...
              params.bell*(abs(cos(params.K*obj.setup.time(1))).*cos(params.K*obj.setup.time(1)) - abs(sin(params.K*obj.setup.time(1))).*sin(params.K*obj.setup.time(1))), ...
              0*obj.setup.time,'md','MarkerSize',5,'LineWidth',2);
        end

        
          
        
        Xhat_est = obj.init.X_est(traj).val([pos_theta, pos_p],:);
        X_est = Xhat_est;
        Xhat = obj.init.X(traj).val([pos_theta, pos_p],:);
        X = Xhat;
        % plot rover        
        plot3(X_est(2,1),X_est(3,1),X_est(4,1),'cd','MarkerSize',5,'LineWidth',2);
        for traj=1:obj.setup.Ntraj 
            plot3(X_est(2,:),X_est(3,:),X_est(4,:),'--','LineWidth',1.5);
            set(gca,'ColorOrderIndex',traj)
            plot3(X(2,:),X(3,:),X(4,:),'LineWidth',1.5);    
        end      
        
        % plot rover - Tag 1      
        for traj=1:obj.setup.Ntraj 
            plot3(X_est(2,:) + (params.dist_tag*sin(X_est(1,:))), ...
                  X_est(3,:) + (-params.dist_tag*cos(X_est(1,:))), ...
                  X_est(4,:),'--','LineWidth',0.5);
            set(gca,'ColorOrderIndex',traj)
            plot3(X(2,:) + (params.dist_tag*sin(X(1,:))), ...
                  X(3,:) + (-params.dist_tag*cos(X(1,:))), ...
                  X(4,:),'--','LineWidth',0.5);
        end
        
        % plot rover - Tag 2      
        for traj=1:obj.setup.Ntraj 
            plot3(X_est(2,:) + (-params.dist_tag*sin(X_est(1,:))), ...
                  X_est(3,:) + (params.dist_tag*cos(X_est(1,:))), ...
                  X_est(4,:),'--','LineWidth',0.5);
            set(gca,'ColorOrderIndex',traj)
            plot3(X(2,:) + (-params.dist_tag*sin(X(1,:))), ...
                  X(3,:) + (params.dist_tag*cos(X(1,:))), ...
                  X(4,:),'--','LineWidth',0.5);
        end
        
        xlabel('X')
        ylabel('Y')
        set(gca,'fontsize', fontsize)

    catch
        close
        fig_count = fig_count -1;
    end
end