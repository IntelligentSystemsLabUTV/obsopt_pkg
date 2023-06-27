function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        % traj
        traj = obs.init.traj;        

        % patrolling on x-y
        T = t(1);
        d = 0.1;
        dzero = 0.995;
        vx = dzero*drive(params.pos_v(1));
        vy = dzero*drive(params.pos_v(2));
        if mod(T,params.freq_u) < 0.25*params.freq_u
            vx = params.amp_ux*(1-exp(-d*mod(T,params.freq_u/4)));            
        elseif mod(T,params.freq_u) < 0.5*params.freq_u
            vy = params.amp_uy*(1-exp(-d*mod(T,params.freq_u/4)));
        elseif mod(T,params.freq_u) < 0.75*params.freq_u
            vx = -params.amp_ux*(1-exp(-d*mod(T,params.freq_u/4)));
        else
            vy = -params.amp_uy*(1-exp(-d*mod(T,params.freq_u/4)));
        end        
        u(1,:) = params.Ku(1)*(vx-drive(params.pos_v(1)));
        u(2,:) = params.Ku(2)*(vy-drive(params.pos_v(2)));
        
        % compute the time index
        for i=1:length(t)
            tdiff = obs.setup.time-t(i);   
            pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
            pos(i) = max(1,pos(i));        
        end  
        ind = pos(1);

        % hills on z
        p_now = drive(params.pos_p(1:2));
        p_est = zeros(1,2);
        p_grid = [params.X_gauss(1,:); params.Y_gauss(:,1)'];
        for i=1:2
            pdiff = p_grid(i,:)-p_now(i);   
            p_est(i) = find(abs(pdiff) == min(abs(pdiff)),1,'first');                
        end
        z_des = params.G_gauss(traj).val(p_est(2),p_est(1));
        z_now = drive(params.pos_p(3));
        zdot_now = drive(params.pos_v(3));

        obs.init.params.z_des_story(:,ind) = z_des;
        obs.init.params.z_now_story(:,ind) = z_now;

        % error        
        e = (z_des-z_now);
        obs.init.params.err(traj).val(:,ind) = e;

        % derivative        
        [edot, obs.init.params.err_der_buffer, obs.init.params.err_der_counter(traj).val] = PseudoDer(params.Ts,...
            obs.init.params.err(traj).val(:,max(1,ind-1)),params.wlen_err,...
            params.buflen_err,params.dim_err,0,0,obs,obs.init.params.err_der_buffer,obs.init.params.err_der_counter(traj).val);
        obs.init.params.err_der(traj).val(:,ind) = edot;

        % integral
        obs.init.params.err_int(traj).val(:,ind) = obs.init.params.err_int(traj).val(:,max(1,ind-1)) + e*params.Ts;
        eint = obs.init.params.err_int(traj).val(:,ind);

        % control
        u(3,:) = -params.Kz(1)*-e -params.Kz(2)*zdot_now + params.Kff*[e edot eint]';

        % rotation control
        u(4) = 0.1;
        u(5) = 0.1;
        u(6) = 0;

        % ony for testing
        u(7,:) = z_des;
        
    end

end