%% plot manipulator trajectory 
function plot_manipulator(obs)
    
    % setup figure
    figure(1)
    hold on    

    % cycle over the trajectories
    for traj=1:obs.setup.Ntraj
        
        % get end effector position - reference
        [~, EE_pos_ref] = measure_manipulator(obs.init.X(traj).val,obs.setup.params,obs.setup.time);
        
        % plot reference trajectory - origin
        pos_origin_ref = reshape(EE_pos_ref(:,1,:),size(EE_pos_ref,1),size(EE_pos_ref,3));
        pos_L1_ref = reshape(EE_pos_ref(:,2,:),size(EE_pos_ref,1),size(EE_pos_ref,3));
        pos_L2_ref = reshape(EE_pos_ref(:,3,:),size(EE_pos_ref,1),size(EE_pos_ref,3));
        
        for i=1:obs.setup.Niter
            x = [pos_origin_ref(1,i),pos_L1_ref(1,i),pos_L2_ref(1,i)];
            y = [pos_origin_ref(2,i),pos_L1_ref(2,i),pos_L2_ref(2,i)];
            plot(x,y,'b--o','LineWidth',0.1);
        end
        
        % get end effector position - real
        [~, EE_pos_real] = measure_manipulator(obs.init.X_est(traj).val,obs.setup.params,obs.setup.time);
        
        % plot reference trajectory - origin
        pos_origin_real = reshape(EE_pos_real(:,1,:),size(EE_pos_real,1),size(EE_pos_real,3));
        pos_L1_real = reshape(EE_pos_real(:,2,:),size(EE_pos_real,1),size(EE_pos_real,3));
        pos_L2_real = reshape(EE_pos_real(:,3,:),size(EE_pos_real,1),size(EE_pos_real,3));
        
        for i=1:obs.setup.Niter
            x = [pos_origin_real(1,i),pos_L1_real(1,i),pos_L2_real(1,i)];
            y = [pos_origin_real(2,i),pos_L1_real(2,i),pos_L2_real(2,i)];
            plot(x,y,'r:o','LineWidth',0.1);
        end
    end
    
    grid on
    box on
    xlabel('x axis');
    ylabel('y axis');
end