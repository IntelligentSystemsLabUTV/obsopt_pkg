function u = control_drone(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % traj
        % traj = obs.init.traj;

        % translation
        u(1) = 0.0005*drive(1) + 0.0005*drive(4); %1*sin(t);%0.0;%
        u(2) = 0.0005*drive(2) + 0.0005*drive(5); %1*cos(t);
        u(3) = 1*drive(3) + 1*drive(6); %0.1;

        % rotation
        u(4) = 0.0;
        u(5) = 0.0;
        u(6) = 0.0;
        
    end

end