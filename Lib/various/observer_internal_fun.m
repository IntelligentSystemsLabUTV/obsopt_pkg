%% compute the cost function
function [obs, V_num, out] = observer_internal_fun(x_grid,params,obs,out,IT,V_num)

    % flag
    BackIterIndex_mem = obs.init.BackIterIndex;
    obs.init.BackIterIndex = 1;

    %%% INTERNA A FUNZIONE %%%
    % init the state, using the for indices IT
    % reset X_est
    obs.init.X_est = 0.*obs.init.X_est;
    tmp_str = 'obs.init.X_est(:,1) = [';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'x_grid(',num2str(i),').val(IT(',num2str(i),')),'];
    end
    tmp_str = [tmp_str(1:end-1), '];'];
    eval(tmp_str);
    
    % reset true X
    startTrue = obs.init.X(:,BackIterIndex_mem);
    obs.init.X = 0.*obs.init.X;
    obs.init.X(:,1) = startTrue;
    

    % simulate the system 
    for i = 1:obs.setup.Niter

        % set current interation in the class
        obs.init.ActualTimeIndex = i;
        obs.init.t = obs.setup.time(i);

        %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
        % forward propagation of the previous estimate
        if(obs.init.ActualTimeIndex > 1)
            % input
            obs.init.params.u = obs.setup.params.input(:,obs.init.ActualTimeIndex-1);

            % true system
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X(:,obs.init.ActualTimeIndex-1));   
            obs.init.X(:,obs.init.ActualTimeIndex) = X.y(:,end);

            % real system
            X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(:,obs.init.ActualTimeIndex-1));
            obs.init.X_est(:,obs.init.ActualTimeIndex) = X.y(:,end);
        end

        %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
        % here the noise is added
        y_meas = obs.setup.measure(obs.init.X(:,obs.init.ActualTimeIndex),obs.init.params);

        %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        obs = obs.observer(obs.init.X_est(:,obs.init.ActualTimeIndex),y_meas);


    end

    % get the ESTIMATED state at init time. This will be done for each IT
    % in the nested for loops, gridding all the x_grid values and computing
    % the cost function in all the points. The observer was used to get
    % BOTH the TRUE state values and GRID state values in the selected
    % buffered down samples
    obs.init.temp_x0 = obs.init.X_est(:,1);
    
    % set the TRUE buffer, used to compare in the cost function
    obs.init.Y_space =  [(out.Nts+out.offset):out.Nts:(out.Nts*(out.Nw)+out.offset)];
    obs.init.Y = obs.init.Y_full_story(:,:,obs.init.Y_space);

    %%% INTERNA A FUNZIONE %%%
    tmp_str = 'V_num(';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'IT(',num2str(i),'),'];
    end
    tmp_str = [tmp_str(1:end-1), ') = obs.cost_function(obs.init.temp_x0);'];
    eval(tmp_str);

    % stop for zero val
    % the grid construction was made in order to always have the true state
    % in the middle. By doing so there wil always be an analytic zero in
    % the centre of the cost function. When this is the case (depending on IT)
    % we want to compute the TRUE trajectory excitation, and save the
    % results. We set this condition only to do it once, as Y remains
    % always the same. 
    if obs.init.temp_x0 == obs.init.X(:,1)
       out.PE_num = trapz(out.Ts,obs.init.Y(1,:).^2);
       out.y_star_num = obs.init.Y(1,:);
       if out.filter
            out.PE_dot_num = trapz(out.Ts,obs.init.Y(2,:).^2);
            out.y_dot_star_num = obs.init.Y(2,:);
       end
    end
    
    % restore backIterIndex
    obs.init.BackIterIndex = BackIterIndex_mem;

end