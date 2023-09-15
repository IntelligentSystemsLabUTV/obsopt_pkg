function u = control_drone(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % traj
        % traj = obs.init.traj;
        k1 = 200;
        k2 = 200;
        % translation
        u(1) = k1*drive(1) + k2*drive(4); %1*sin(t);%0.0;%
        u(2) = k1*drive(2) + k2*drive(5); %1*cos(t);
        u(3) = k1*drive(3) + k2*drive(6); %0.1;

        % rotation
        u(4) = 0.0;
        u(5) = 0.0;
        u(6) = 0.0;
        
    end

end