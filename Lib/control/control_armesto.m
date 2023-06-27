function u = control_armesto(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % traj
        % traj = obs.init.traj;

        % translation
        u(1) = 0.1*sin(t);
        u(2) = 0.1*cos(t);
        u(3) = 0.1;

        % rotation
        u(6) = 1;
        
    end

end