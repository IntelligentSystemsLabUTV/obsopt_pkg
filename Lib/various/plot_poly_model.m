%%
function plot_poly_model(obs_fast,init_flag,paramCloud)

    if ~paramCloud
        % setup
        if ~init_flag
            plot_interp(obs_fast,obs_fast.setup.params.input_OCV,1,...
                [obs_fast.setup.params.alpha_Voc,obs_fast.setup.params.beta_Voc,obs_fast.setup.params.gamma_Voc,obs_fast.setup.params.delta_Voc],0,1);

            plot_interp(obs_fast,obs_fast.setup.params.input_R0,2,...
                [obs_fast.setup.params.alpha_R0,obs_fast.setup.params.beta_R0,obs_fast.setup.params.gamma_R0,obs_fast.setup.params.delta_R0],0,2);

            plot_interp(obs_fast,obs_fast.setup.params.input_R1,3,...
                [obs_fast.setup.params.alpha_R1,obs_fast.setup.params.beta_R1,obs_fast.setup.params.gamma_R1,obs_fast.setup.params.delta_R1],0,3);

            plot_interp(obs_fast,obs_fast.setup.params.input_C1,4,...
                [obs_fast.setup.params.alpha_C1,obs_fast.setup.params.beta_C1,obs_fast.setup.params.gamma_C1,obs_fast.setup.params.delta_C1],0,4);
        else    
            % init
            plot_interp(obs_fast,obs_fast.init.params.input_OCV,1,...
                [obs_fast.init.params.alpha_Voc,obs_fast.init.params.beta_Voc,obs_fast.init.params.gamma_Voc,obs_fast.init.params.delta_Voc],0,1);

            plot_interp(obs_fast,obs_fast.init.params.input_R0,2,...
                [obs_fast.init.params.alpha_R0,obs_fast.init.params.beta_R0,obs_fast.init.params.gamma_R0,obs_fast.init.params.delta_R0],0,2);

            plot_interp(obs_fast,obs_fast.init.params.input_R1,3,...
                [obs_fast.init.params.alpha_R1,obs_fast.init.params.beta_R1,obs_fast.init.params.gamma_R1,obs_fast.init.params.delta_R1],0,3);

            plot_interp(obs_fast,obs_fast.init.params.input_C1,4,...
                [obs_fast.init.params.alpha_C1,obs_fast.init.params.beta_C1,obs_fast.init.params.gamma_C1,obs_fast.init.params.delta_C1],0,4);
        end
    else
        figure(1)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_OCV,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est_runtime.val(3,:),'ro');
        
        figure(2)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_R0,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est_runtime.val(4,:),'ro');
        
        figure(3)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_R1,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est_runtime.val(5,:),'ro');
        
        figure(4)
        grid on
        hold on
        plot(obs_fast.init.params.input_soc,obs_fast.setup.params.input_C1,'LineWidth',2)
        plot(obs_fast.init.X_est.val(1,:),obs_fast.init.X_est_runtime.val(6,:),'ro');
    end
   

end