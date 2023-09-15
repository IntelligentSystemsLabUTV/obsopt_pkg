%% SIMULATION_GENERAL_V3
% file: simulation_general_v3.m
% author: Federico Oliva
% date: 10/01/2022
% description: function to setup and use the MHE out.observer on general model
% INPUT: none
% OUTPUT: params,out.obs
% plot results for control design
function plot_drone(obj,varargin)
    
    close all

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
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_p(i),:),'--','LineWidth',1);                                                                                                  
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
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_v(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_v(i),:),'--','LineWidth',1);                                                                                    
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
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_acc(i),:),'--','LineWidth',1);                                                                                  
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
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_bias(i),:),'--','LineWidth',1);                                                                                   
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

    %%% plot Quaternion estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Quaternion estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_quat)
            subplot(length(params.pos_quat),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_quat),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_quat(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_quat(i),:),'--','LineWidth',1);                                                                         
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['q_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est')            
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end

    %%% plot omega estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Omega estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_omega)
            subplot(length(params.pos_omega),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_omega),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_omega(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_omega(i),:),'--','LineWidth',1);                                                                                   
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['\omega_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est') 
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end

 
    %%% plot alpha estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Alpha estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_alpha)
            subplot(length(params.pos_alpha),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_alpha),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_alpha(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_alpha(i),:),'--','LineWidth',1);                                                                                
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['\alpha_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est')     
        xlabel(['time [s]'])
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end

    %%% plot bias vel estimation
    try
        fig_count = fig_count+1;
        figure(fig_count)            
        sgtitle('Bias vel estimation')
        ax = zeros(1,3);
        for i=1:length(params.pos_bias_v)
            subplot(length(params.pos_bias_v),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_bias_v),1,i);     
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,obj.init.X(traj).val(params.pos_bias_v(i),:),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,obj.init.X_est_runtime(traj).val(params.pos_bias_v(i),:),'--','LineWidth',1);                                                                         
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['bv_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Est')            
        xlabel(['time [s]'])    
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end
    
    %%%% plot uwb measure data %%%%   
    try
        fig_count = fig_count+1;
        figure(fig_count) 
        txt = "uwb measure " + num2str((1/params.UWB_samp)*100) + "Hz - Noise: " + num2str(params.noise_mat(params.pos_uwb_out(1),2)) + " m";
        sgtitle(txt)
        ax = zeros(1,3);
        for i=1:length(params.pos_uwb_out)
            subplot(length(params.pos_uwb_out),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_uwb_out),1,i);    

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_uwb_out(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_uwb_out(i),:)),'--','LineWidth',1);          

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_uwb_out(i),obj.init.temp_time),1,length(WindowTime));
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




    try
        fig_count = fig_count+1;
        figure(fig_count) 
        txt = "uwb derivative ";
        sgtitle(txt)
        ax = zeros(1,3);
        for i=1:length(params.pos_uwb_out_der)
            subplot(length(params.pos_uwb_out_der),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_uwb_out_der),1,i);    

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_uwb_out_der(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_uwb_out_der(i),:)),'--','LineWidth',1);          

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_uwb_out_der(i),obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'o','MarkerSize',5);
                catch 
                    disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
                end
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['p_dot_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Meas')            
        xlabel(['time [s]'])   
        %linkaxes(ax);
    catch
        close
        fig_count = fig_count -1;
    end





   %%%% plot cam measure data %%%%   
    try
        fig_count = fig_count+1;
        figure(fig_count)  
        txt = "cam measure " + num2str((1/params.CAM_samp)*100) + "Hz - Noise: " + num2str(params.noise_mat(params.pos_cam_out(1),2)) + " m";
        sgtitle(txt)
        ax = zeros(1,3);
        for i=1:length(params.pos_cam_out)
            subplot(length(params.pos_cam_out),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_cam_out),1,i);    

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_cam_out(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_cam_out(i),:)),'--','LineWidth',1);          

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_cam_out(i),obj.init.temp_time),1,length(WindowTime));
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

    try
        %%% drone trajectory
        fig_count = fig_count+1;
        figure(fig_count)
        hold on
        grid on

        % plot drone     
        pos_p = obj.init.params.pos_p;
        for traj=1:obj.setup.Ntraj
            plot3(obj.init.X_est_runtime(traj).val(pos_p(1),:),obj.init.X_est_runtime(traj).val(pos_p(2),:),obj.init.X_est_runtime(traj).val(pos_p(3),:),'--','LineWidth',1.5);
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


    try
        fig_count = fig_count+1;
        figure(fig_count) 
        txt = "cam derivative ";
        sgtitle(txt)
        ax = zeros(1,3);
        for i=1:length(params.pos_cam_out_der)
            subplot(length(params.pos_cam_out_der),1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(length(params.pos_cam_out_der),1,i);    

            % down sampling instants
            WindowTime = obj.setup.time(obj.init.temp_time);
            
            for traj=1:obj.setup.Ntraj            
                plot(obj.setup.time,squeeze(obj.init.Ytrue_full_story(traj).val(1,params.pos_cam_out_der(i),:)),'LineWidth',2);
                set(gca,'ColorOrderIndex',traj)
                plot(obj.setup.time,squeeze(obj.init.Y_full_story(traj).val(1,params.pos_cam_out_der(i),:)),'--','LineWidth',1);          

                set(gca,'ColorOrderIndex',traj)
                % plot target values    
                try
                    data = reshape(obj.init.Y_full_story(traj).val(1,obj.setup.params.pos_cam_out_der(i),obj.init.temp_time),1,length(WindowTime));
                    plot(WindowTime,data,'o','MarkerSize',5);
                catch 
                    disp('CHECK T_END OR AYELS CONDITION - LOOKS LIKE NO OPTIMISATION HAS BEEN RUN')
                end
            end
            
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['p_dot_',num2str(obj.setup.plot_vars(i))])
        end
        legend('True','Meas')            
        xlabel('time [s]')   
    catch
        close
        fig_count = fig_count -1;
    end


    try
        fig_count = fig_count+1;
        figure(fig_count) 
        txt = "Gamma Story";
        sgtitle(txt)
        %ax = zeros(1,size(obs.init.params.gamma_story,1));
        for i=1:size(params.gamma_story,2)
            subplot(size(params.gamma_story,2),1,i);
            hold on
            grid on
            box on 
            plot(params.gamma_story(:,i),'LineWidth',2);
        end
    catch
        close
        fig_count = fig_count -1;
    end

end