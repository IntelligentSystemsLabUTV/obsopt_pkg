%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE out.observer on general model
% INPUT: none
% OUTPUT: params,out.obs
% plot results for control design
function plot_3Dobject_realdata(obj,varargin)
    
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

    %%% plot position estimation
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
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_v(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est(traj).val(params.pos_v(i),:),'--','LineWidth',1);                                                                                    
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
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_acc(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est(traj).val(params.pos_acc(i),:),'--','LineWidth',1);                                                                                  
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

    try
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
        % plot anchors
        P_a(1,:) = obj.init.params.pos_anchor(1):3:obj.init.params.pos_anchor(end);
        P_a(2,:) = obj.init.params.pos_anchor(2):3:obj.init.params.pos_anchor(end); 
        P_a(3,:) = obj.init.params.pos_anchor(3):3:obj.init.params.pos_anchor(end); 
        for i=1:obj.setup.params.Nanchor
            plot3(obj.init.X_est(1).val(P_a(1,i),:),obj.init.X_est(1).val(P_a(2,i),:),obj.init.X_est(1).val(P_a(3,i),:),'ko','MarkerSize',10);
        end
        % plot rover        
        for traj=1:obj.setup.Ntraj 
            plot3(obj.init.X_est(traj).val(pos_p(1),:),obj.init.X_est(traj).val(pos_p(2),:),obj.init.X_est(traj).val(pos_p(3),:),'--','LineWidth',1.5);
            set(gca,'ColorOrderIndex',traj)
            plot3(obj.init.X(traj).val(pos_p(1),:),obj.init.X(traj).val(pos_p(2),:),obj.init.X(traj).val(pos_p(3),:),'LineWidth',1.5);    
        end
        xlabel('X')
        ylabel('Y')
        set(gca,'fontsize', fontsize)

    catch
        close
        fig_count = fig_count -1;
    end
end