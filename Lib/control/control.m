function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % 2nd order system
        u(1,:) = params.eps_u*(-2*params.rhox*params.wnx*drive(2,:) - 1*params.wnx^2*drive(1,:));
        u(2,:) = params.eps_u*(-2*params.rhoy*params.wny*drive(7,:) - 1*params.wny^2*drive(6,:));

        % sine on x - drift on y
%         u(1,:) = params.eps_u*(-2*params.rhox*params.wnx*drive(2) - 1*params.wnx^2*drive(1));
%         u(2,:) = params.amp_uy;

        % vines  
%         T = t(1);
%         vx = 0;
%         vy = 0;
%         if mod(T,params.freq_u) < 0.25*params.freq_u
%             vx = params.amp_ux;
%         elseif mod(T,params.freq_u) < 0.5*params.freq_u
%             vy = params.amp_uy;
%         elseif mod(T,params.freq_u) < 0.75*params.freq_u
%             vx = -params.amp_ux;
%         else
%             vy = params.amp_uy;
%         end
%         u(1,:) = params.Ku*(vx-drive(2));
%         u(2,:) = params.Ku*(vy-drive(7));
    end

    %Test
    if any(u > 50)
a=1;
    end

end