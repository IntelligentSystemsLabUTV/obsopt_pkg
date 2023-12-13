%%
function y_noise = generate_measure(obs,params)

    obs.init.traj = 1;

    for i = 2:obs.setup.Niter
        
        % define the time span of the integration
        startpos = i-1;
        stoppos = i;
        tspan = obs.setup.time(startpos:stoppos);   
        
        % true system - correct initial condition and no noise
        % considered
        X = odeDD(@(t,x)obs.setup.model_reference(t, x, obs.setup.params, obs), tspan, obs.init.X(1).val(:,startpos),params.odeset); 
        obs.init.X(1).val(:,startpos:stoppos) = [X.y(:,1),X.y(:,end)];

        % here the noise is noise added aggording to noise_spec
        Ytrue_full_story(1).val(1,:,i) = obs.setup.measure_reference(obs.init.X(1).val(:,i),obs.init.params,obs.setup.time(i));
        obs.init.Ytrue_full_story(1).val(1,:,i) =  Ytrue_full_story(1).val(1,:,i);
        CurrentMeas = reshape(obs.init.Ytrue_full_story(1).val(1,:,i),obs.setup.dim_out,1);
        noise_story(1).val(:,i) = obs.setup.noise*(obs.setup.noise_mu(obs.setup.observed_state)  + obs.setup.noise_std(obs.setup.observed_state).*CurrentMeas.*randn(obs.setup.dim_out,1));
        y_noise(:,i) =  CurrentMeas + noise_story(1).val(:,i);
    end

end