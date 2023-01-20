function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % 2nd order system
%         u(1,:) = params.Ax(1)*sin(params.wnx(1).*t + params.phi(1)) + params.Ax(2)*sin(params.wnx(2).*t + params.phi(1));
%         u(2,:) = params.Ay(1)*sin(params.wny(1).*t + params.phi(2)) + params.Ay(2)*sin(params.wny(2).*t + params.phi(2));
        
        % sum of sines (position)
        target(1,:) = params.Ax(1)*sin(params.wnx(1).*t + params.phi(1)) + params.Ax(2)*sin(params.wnx(2).*t + params.phi(1));
        target(2,:) = params.Ay(1)*sin(params.wny(1).*t + params.phi(2)) + params.Ay(2)*sin(params.wny(2).*t + params.phi(2));
        u(1,:) = params.Ku(1)*(target(1,:)-drive(2,:));
        u(2,:) = params.Ku(2)*(target(2,:)-drive(7,:));
        

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

end