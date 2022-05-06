%% test the trained control 
function test = test_manipulator(obs,Ntraj)

    % define time
    test.t0 = 0;
    test.tend = 5;
    test.tstep = obs.setup.Ts;
    test.time = test.t0:test.tstep:test.tend;
    test.Niter = length(test.time);
    test.Ntraj = Ntraj;
    test.bound_delta_x = 5e-1*[-1,1]*1;
    test.bound_delta_x_dot = 1e-2*[-1,1]*0;
    
    for traj=1:test.Ntraj
        % init 
        test.X(traj).val(:,1) = obs.init.X(1).val(1:obs.init.params.dim_state,1);
        test.X_est(traj).val(:,1) = obs.init.X(1).val(1:obs.init.params.dim_state,1); 
%         test.X_est(traj).val(:,1) = obs.init.X_est(1).val(1:obs.init.params.dim_state,1); 
        
        % rand vals
        rand_vals = 1*[unifrnd(test.bound_delta_x(1),test.bound_delta_x(2),2,1); unifrnd(test.bound_delta_x_dot(1),test.bound_delta_x_dot(2),2,1)];
        
        % init traj
        test.X_est(traj).val(1:obs.init.params.dim_state,1) = test.X_est(traj).val(1:obs.init.params.dim_state,1) + rand_vals;

        for i = 1:test.Niter

            if i>1
                
                startpos = i-1;
                stoppos = i;
                tspan = test.time(startpos:stoppos);  

                % true system
                X = obs.setup.ode(@(t,x)obs.setup.model_reference(t, x, obs.init.params), tspan, test.X(traj).val(:,startpos),obs.setup.ode);   
                test.X(traj).val(:,i) = X.y(1:obs.init.params.dim_state,end);

                % real system
                X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan, test.X_est(traj).val(:,startpos),obs.setup.ode);
                test.X_est(traj).val(:,i) = X.y(1:obs.init.params.dim_state,end);
            end

        end
    end    
    
end