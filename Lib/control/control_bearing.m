%% control bearing
function [u, params_out] = control_bearing(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));    
    params_out = params;

    if params.input_enable

        % traj
        traj = obs.init.traj;                   
        
        % times
        T = t(1);
        Tend = obs.setup.time(end);                 
                
        % ref point - checkpoints
          
        % ref point - ellipse
        if params.shape == 1
            x_des = [0 params.aell*cos(params.K*T) params.bell*sin(params.K*T)];
        else
            % ref point - rectangle
            x_des = [0, ...
                 params.aell*(abs(cos(params.K*T))*cos(params.K*T) + abs(sin(params.K*T))*sin(params.K*T)), ...
                 params.bell*(abs(cos(params.K*T))*cos(params.K*T) - abs(sin(params.K*T))*sin(params.K*T))];
        end
        
        % derivative
        [params_out.xrefder(:,end+1), params_out.xrefderbuf, params_out.xrefdercount] = PseudoDer( ...
        params.Ts,x_des(2:3),params.cder,params.dder,2,0,0,obs,params_out.xrefderbuf,params_out.xrefdercount); 
        xdot = params_out.xrefder(1,end);
        ydot = params_out.xrefder(2,end);
        
        % general storing stuff
        obs.init.params.xrefder = params_out.xrefder;
        obs.init.params.xrefderbuf = params_out.xrefderbuf;
        obs.init.params.xrefdercount = params_out.xrefdercount;
        
        
        % convert ref point - Martinez
        xhat_des = x_des;            
        
        % position error - x
        epx = (drive(params.pos_p(1)) - xhat_des(2));
        u0 =  -params.Kxy(1)*epx + xdot;
        
        % position error - y
        epy = (drive(params.pos_p(2)) - xhat_des(3));            
        v0 =  -params.Kxy(1)*epy + ydot;
        
        % bearing error - Martinez
        x2star = v0/u0;        
        etheta = tan(drive(params.pos_theta)) - x2star;
        
        %%%%% patrolling on xy  - Martinez %%%%%%
        u(1,1) = u0;
        u(2,1) = -params.Kxy(2)*sign(etheta);        
        
        % saturation
        u(1,1) = min(max(u(1,1),-params.uSat),params.uSat);
        u(2,1) = min(max(u(2,1),-params.uSat),params.uSat);
        
        
        %%%%%% hills on z %%%%%%
        % compute the time index
        for i=1:length(t)
            tdiff = obs.setup.time-t(i);   
            pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
            pos(i) = max(1,pos(i));        
        end  
        ind = pos(1);
        
        % desired height
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

        % store
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

        % ony for testing - save desired z
        u(4,:) = z_des;
        
    end

end