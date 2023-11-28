function u = control_drone(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    
    if params.input_enable

        % traj
        % traj = obs.init.traj;
        
        % k1 = params.gamma(4);
        % k2 = params.gamma(5);

       
        % drive = error
        % drive(1:3)    = position              (y-pos)
        % drive(4:6)    = orientation           (y-eul)
        % drive(7:9)    = velocity              (y-vel)
        % drive(10:12)  = omega                 (y-omega)
        % drive(13:15)  = position measured     (pos)
        % drive(16:18)  = orientation measured  (eul)

        %Theta is the pitch that let the drone move along x (angle along y)
        use_control = 1;
        Kp = 0.01;
        Kd = 1;
        Kp_z            = use_control   *   0.05;
        Kd_z            = use_control   *   0.1;
        Kp_phi_or       = use_control   *   5;
        Kd_phi_omega    = use_control   *   0.1;
        Kp_theta_or     = use_control   *   5;
        Kd_theta_omega  = use_control   *   2;
        Kp_psi_or       = use_control   *   5;
        Kd_psi_omega    = use_control   *   1;
        Kp_phi_pos      = use_control   *   Kp;
        Kd_phi_vel      = use_control   *   Kd;
        Kp_theta_pos    = use_control   *   Kp;
        Kd_theta_vel    = use_control   *   Kd;

        Mcontrol = [1,1,1,1;0,-1,0,1;-1,0,1,0;-1,1,-1,1];

        Thrust      = (params.g*params.m)/(cos(drive(17))*cos(drive(16))) + Kp_z*(drive(3)) + Kd_z*(drive(9));% Kp_z*(drive(9));% /(cos(drive(17))*cos(drive(16)))
        Tau_phi     = 0;%3.8*drive(10)+5.15*drive(4)+2.95*drive(8)/Thrust +0.6*drive(2)/Thrust;%Kd_phi_omega*drive(11);% - Kd_phi_vel*drive(7) ;% Kp_phi_or*drive(5) 
        Tau_theta   = 3.8*drive(11)+5.15*drive(5)+2.95*drive(7)/Thrust +0.6*drive(1)/Thrust; %Kd_theta_omega*drive(7) + Kp_phi_pos*drive(1);%Kd_theta_omega*drive(11) - Kd_theta_vel*drive(8) - Kp_theta_pos*drive(2);% Kp_theta_or*drive(4)
        Tau_psi     = 0;%Kd_psi_omega*drive(12);%Kp_psi_or*drive(6) + Kd_psi_omega*drive(12);%
        
        U = pinv(Mcontrol)*[Thrust;Tau_phi;Tau_theta;Tau_psi;];

        % thrust
        u(1) = U(1);
        u(2) = U(2);
        u(3) = U(3);
        u(4) = U(4);

        obs.init.params.control_story(round((t*100)+1),:) = u';
        
        
    end

end