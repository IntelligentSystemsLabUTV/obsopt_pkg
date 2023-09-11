function u = control_drone(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % traj
        % traj = obs.init.traj;

        % translation
        u(1) = 0.0;%1*sin(t);%0.0;%
        u(2) = 0.0;%1*cos(t);
        u(3) = 0.0;%0.1;

        % rotation
        u(4) = 0.0;
        u(5) = 0.0;
        u(6) = 0.0;
        
    end

end