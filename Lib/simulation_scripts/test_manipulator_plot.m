%%
function test_manipulator_plot(test,obs)

    % PLOT section
    
    % state
    figure
    title('Manipulator jointspace')
    for traj=1:test.Ntraj
        for i=1:obs.setup.plot_vars
            subplot(obs.setup.plot_vars,1,i);
            hold on
            grid on
            box on

            plot(test.time,test.X(traj).val(i,:),'k');
            plot(test.time,test.X_est(traj).val(i,:),'b--');            
            legend('True','Est')        
        end
    end
    
    % plot workspace
    figure
    hold on
    grid on
    box on
%     title('Manipulator workspace')
    for traj=1:test.Ntraj
        % target
        [~, yall] = measure_manipulator(test.X(traj).val(:,1),obs.init.params);
        plot(yall(1,:),yall(2,:),'b+-','LineWidth',0.1);
        test.dist_true(traj).val(1,1) = norm(yall(:,2)-yall(:,1));
        test.dist_true(traj).val(2,1) = norm(yall(:,3)-yall(:,2));          

        % start - controlled 
        [~, yall] = measure_manipulator(test.X_est(traj).val(:,1),obs.init.params);
        plot(yall(1,:),yall(2,:),'ro-','LineWidth',0.1);

        % arrival - controlled 
        [~, yall] = measure_manipulator(test.X_est(traj).val(:,end),obs.init.params);
        plot(yall(1,:),yall(2,:),'gs-','LineWidth',2);
    end

end