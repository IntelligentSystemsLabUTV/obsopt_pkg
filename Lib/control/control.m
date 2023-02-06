function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % PWM 3 levels

        % 2nd order system
%         u(1,:) = -2*params.rhox(1)*params.wnx(obs.init.traj)*drive(2) -params.wnx(obs.init.traj)^2*drive(1);
%         u(2,:) = -2*params.rhoy(1)*params.wny(obs.init.traj)*drive(6) -params.wny(obs.init.traj)^2*drive(5);
        
        % sum of sines (position)
%         target(1,:) = exp(-params.rhox.*t).*(params.Ax(1)*sin(params.wnx(1).*t + params.phi(1)) + params.Ax(2)*sin(params.wnx(2).*t + params.phi(1)));
%         target(2,:) = exp(-params.rhoy.*t).*(params.Ay(1)*sin(params.wny(1).*t + params.phi(2)) + params.Ay(2)*sin(params.wny(2).*t + params.phi(2)));
%         u(1,:) = params.Ku(1)*(target(1,:)-drive(1,:));
%         u(2,:) = params.Ku(2)*(target(2,:)-drive(6,:));
        

        % vines  
        T = t(1);
        vx = 0;
        vy = 0;
        if mod(T,params.freq_u) < 0.25*params.freq_u
            vx = params.amp_ux;
        elseif mod(T,params.freq_u) < 0.5*params.freq_u
            vy = params.amp_uy;
        elseif mod(T,params.freq_u) < 0.75*params.freq_u
            vx = -params.amp_ux;
        else
            vy = params.amp_uy;
        end
        u(1,:) = params.Ku(1)*(vx-drive(2));
        u(2,:) = params.Ku(2)*(vy-drive(6));

        % Volterra Lotka
%         u(1,:) = params.K1*(drive(1)-params.target(1));
%         u(2,:) = params.K2*(drive(1)-params.target(1));
%         % sat
%         u(1,:) = min(max(-params.umax,u(1,:)),params.umax);
%         u(2,:) = min(max(-params.umax,u(2,:)),params.umax);
    end

end