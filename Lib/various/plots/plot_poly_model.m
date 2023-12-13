%%
function plot_poly_model(obs_fast,init_flag,paramCloud,final_flag)

    fontsize=20;        

    % poly model
    if init_flag   
        Shift = (obs_fast.setup.w+1)*obs_fast.setup.Nts;
        for iter=Shift:obs_fast.setup.Nts:obs_fast.setup.Niter
            Zdiff = obs_fast.init.X_est_runtime.val(1,iter)-sort(obs_fast.init.cloud_X(1,:));
            pos = find(abs(Zdiff) == min(abs(Zdiff)),1,'first'); 
            scale = 40;
            range = max(1,pos-ceil(size(obs_fast.init.cloud_X,2)/scale)):max(size(obs_fast.init.cloud_X,2),pos+ceil(size(obs_fast.init.cloud_X,2)/scale));
            try
                plot_interp(obs_fast,obs_fast.init.params.input_OCV,1,obs_fast.init.X_est_runtime.val([7 11 15 19],iter),1,1,range);
                plot_interp(obs_fast,obs_fast.init.params.input_R0,2,obs_fast.init.X_est_runtime.val([8 12 16 20],iter),2,1,range);
                plot_interp(obs_fast,obs_fast.init.params.input_R1,3,obs_fast.init.X_est_runtime.val([9 13 17 21],iter),3,1,range);
                plot_interp(obs_fast,obs_fast.init.params.input_C1,4,obs_fast.init.X_est_runtime.val([10 14 18 22],iter),4,1,range);
            catch
               ARARMAX = 1; 
            end
        end       
    end
    
    % initial guess
    iter = 1;
    plot_interp(obs_fast,obs_fast.setup.params.input_OCV,1,obs_fast.init.X_est_runtime.val([7 11 15 19],iter),1,0,0);
    plot_interp(obs_fast,obs_fast.setup.params.input_R0,2,obs_fast.init.X_est_runtime.val([8 12 16 20],iter),2,0,0);
    plot_interp(obs_fast,obs_fast.setup.params.input_R1,3,obs_fast.init.X_est_runtime.val([9 13 17 21],iter),3,0,0);
    plot_interp(obs_fast,obs_fast.setup.params.input_C1,4,obs_fast.init.X_est_runtime.val([10 14 18 22],iter),4,0,0);
    
    if final_flag
        iter = obs_fast.setup.Niter;
        range = 1:obs_fast.setup.Niter;
        plot_interp(obs_fast,obs_fast.init.params.input_OCV,1,obs_fast.init.X_est.val([7 11 15 19],iter),1,0,range);
        plot_interp(obs_fast,obs_fast.init.params.input_R0,2,obs_fast.init.X_est.val([8 12 16 20],iter),2,0,range);
        plot_interp(obs_fast,obs_fast.init.params.input_R1,3,obs_fast.init.X_est.val([9 13 17 21],iter),3,0,range);
        plot_interp(obs_fast,obs_fast.init.params.input_C1,4,obs_fast.init.X_est.val([10 14 18 22],iter),4,0,range);        
    end
    
    % param cloud
    if paramCloud
        figure(1)
        set(gca,'fontsize', fontsize)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.OCV,'LineWidth',2)
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.OCV_nominal,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est.val(3,:),'co');
        legend('measurements','fit data','initial fit','nominal','real','estimated');
        
        figure(2)
        set(gca,'fontsize', fontsize)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.R0,'LineWidth',2)
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.R0_nominal,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est.val(4,:),'co');
        legend('measurements','fit data','initial fit','nominal','real','estimated');
        
        figure(3)
        set(gca,'fontsize', fontsize)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.R1,'LineWidth',2)
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.R1_nominal,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est.val(5,:),'co');
        legend('measurements','fit data','initial fit','nominal','real','estimated');
        
        figure(4)
        set(gca,'fontsize', fontsize)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.C1,'LineWidth',2)
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_data.C1_nominal,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est.val(6,:),'co');
        legend('measurements','fit data','initial fit','nominal','real','estimated');
    end
   

end