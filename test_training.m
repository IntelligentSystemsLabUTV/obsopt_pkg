%% test the trained control 
function test = test_training(obs)

    % define time
    t0 = 0;
    tend = 5;
    tstep = 5e-2;
    time = t0:tstep:tend;
    Niter = length(time);
    traj = 1;
    
    % init 
    temp.xP(:,1) = 0*obs.init.X(1).val(1:obs.init.params.dim_state,1);
    temp.xM(:,1) = 0*obs.init.X(1).val(1:obs.init.params.dim_state,1);
    obs.init.params.r_story = 0*obs.init.params.r_story;
    obs.init.params.r_story(:,Niter+1:end) = [];
    
    for i = 1:Niter
        
        % save used input
        if (mod(i,10) == 0) || (i==1)
            obs.init.params.r_story(:,i) = randi([-3,3],3,1);
        else
            obs.init.params.r_story(:,i) = obs.init.params.r_story(:,i-1);
        end
        obs.init.params.ActualTimeIndex = i;
        
        if i>1
            obs.init.params.u = obs.setup.params.input(obs.init.t,temp.xM(:,i-1),obs.init.params);

            % true system
            X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, obs.init.params), [t0 tstep], temp.xM(:,i-1),obs.setup.ode);   
            temp.xP(:,i) = X.y(1:obs.init.params.dim_state,end);

            % real system
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), [t0 tstep], temp.xP(:,i-1),obs.setup.ode);
            temp.xM(:,i) = X.y(1:obs.init.params.dim_state,end);
        end
               
    end
    
    % PLOT section
    figure(1)
    for i=1:obs.setup.plot_vars
        subplot(obs.setup.plot_vars,1,i);
        hold on
        grid on
        box on

        plot(time,obs.init.params.r_story(i,:),'k');
        plot(time,temp.xM(i,:),'b--');            
        plot(time,temp.xP(i,:),'r.');
        legend('True','Est')        
    end
    
    % assign output
    test.t = time;
    
end