%% MODEL_ROVER
% file: model_rover.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the dynamics equation of a rover
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function [x_dot, x] = model_rover(tspan,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    % use input from reference
    try
        params.u = obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos(1)));
        obs.init.input_story(obs.init.traj).val(:,pos(1)) = params.u(:,1);   
    catch
        obs.init.input_story(obs.init.traj).val(:,pos(1)) = 0;
    end

    %%%%%%%%%%% HYBRID OBSERVER MODEL %%%%%%%%%%%%%

    if (params.hyb) && ~params.dryrun

        % meas available
        y = obs.init.Y_full_story(obs.init.traj).val(1,:,pos(1));
        a = y(params.pos_acc_out);     
        xp = x;
    
        if params.sferlazza == 0
            % Jump - only on the UWB
            if (mod(pos(1),params.UWB_samp) == 0)
        
                % adjacency matrix
                for dim=1:params.space_dim
                    Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
                end
        
                %%% TEST %%%
                p_jump = obs.init.params.p_jump(obs.init.traj).val(:,pos(1)/params.UWB_samp);
                p_jump_der = obs.init.params.p_jump_der(obs.init.traj).val(:,pos(1)/params.UWB_samp);

                % stability analysis - position
                if 1
                    Ae = params.fAe;
                    AeEXP = expm(Ae*params.Ts*params.UWB_samp);                            
                    Aed = double(simplify(params.fAed(params.theta(1), params.theta(2), params.theta(3))));
                    AeJump = (eye(9) - Aed);
                    MONOTROMIC = AeJump*AeEXP;
                    obs.init.params.EIGPOS(:,pos(1)) = sort(real(eig(MONOTROMIC)));
                end
                
                % jump map - x
                xp(1) = x(1) + params.theta(1)*(p_jump(1)-x(1));
                xp(2) = x(2) + params.theta(2)*(p_jump(1)-x(1));
                xp(3) = x(3) + params.theta(3)*(p_jump(1)-x(1));
                xp(4) = x(4);
                        
                % jump map - y
                xp(5) = x(5) + params.theta(1)*(p_jump(2)-x(5));
                xp(6) = x(6) + params.theta(2)*(p_jump(2)-x(5));
                xp(7) = x(7) + params.theta(3)*(p_jump(2)-x(5));
                xp(8) = x(8);               
    
                % jump map - z
                xp(9) = x(9) + params.theta(1)*(p_jump(3)-x(9));
                xp(10) = x(10) + params.theta(2)*(p_jump(3)-x(9));
                xp(11) = x(11) + params.theta(3)*(p_jump(3)-x(9));
                xp(12) = x(12);   

                x = xp;
            end   
            
        else

            % Jump - only on the UWB
            if (mod(pos(1),params.UWB_samp) == 0)
        
                % adjacency matrix
                for dim=1:params.space_dim
                    Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
                end
        
                %%% TEST %%%
                p_jump = obs.init.params.p_jump(obs.init.traj).val(:,pos(1)/params.UWB_samp);
                p_jump_der = obs.init.params.p_jump_der(obs.init.traj).val(:,pos(1)/params.UWB_samp);            
                
                % jump map - x
                range = params.range_sfer_jump(1,:);                
                xref = [p_jump(1)];
                e = xref-params.Cproj*x(range);                
                xp(range) = x(range)+params.Ksfer*e;
        
                % jump map - y
                range = params.range_sfer_jump(2,:);                
                xref = [p_jump(2)];
                e = xref-params.Cproj*x(range);                
                xp(range) = x(range)+params.Ksfer*e;
    
                % jump map - z                
                range = params.range_sfer_jump(3,:);                
                xref = [p_jump(3)];
                e = xref-params.Cproj*x(range);                
                xp(range) = x(range)+params.Ksfer*e;

                x = xp;
            end                

        end 

        % flow map - x
        range = params.range_sfer_flow(1,:);
        x_dot(range) = params.Asfer*x(range) + params.Bsfer*a(1);

        % flow map - y
        range = params.range_sfer_flow(2,:);
        x_dot(range) = params.Asfer*x(range) + params.Bsfer*a(2);

        % flow map - z
        range = params.range_sfer_flow(3,:);
        x_dot(range) = params.Asfer*x(range) + params.Bsfer*a(3);

    elseif params.dryrun

    %%%%%%%%%%%%% REFERENCE MODEL %%%%%%%%%
        x_dot = obs.setup.model_reference(tspan,x,params,obs);

    else

    %%%%%%%%%%%%% ERROR %%%%%%%%%
        error('what do you wanna integrate mate?')

    end

    %% model dynamics - quaternion

    % hyb observer
    if (mod(pos(1),params.UWB_samp) == 0)

        xp = x;

        % meas available
        y = obs.init.Y_full_story(obs.init.traj).val(1,:,pos(1));
        q_jump = obs.init.params.q_jump(obs.init.traj).val(:,pos(1)/params.UWB_samp)';
        % q_jump = y(params.pos_quat_out);

        % from quaternion to RPY
        [yaw, pitch, roll]  = quat2angle(q_jump);
        roll =  y(params.pos_eul_out(1));
        pitch = y(params.pos_eul_out(2));
        yaw =   y(params.pos_eul_out(3));
        [yawhat, pitchhat, rollhat]  = quat2angle(xp(params.pos_quat)');
        err = [wrapTo4PiRound(roll-rollhat), wrapTo4PiRound(pitch-pitchhat), wrapTo4PiRound(yaw-yawhat)];

        % angle jump map
        rollhatP = rollhat + params.gamma(1)*err(1);
        pitchhatP = pitchhat + params.gamma(2)*err(2);
        yawhatP = yawhat + params.gamma(3)*err(3);
        xp(params.pos_quat) = angle2quat(yawhatP, pitchhatP, rollhatP);

        % quat jump
        % err = quatmultiply(quatinv(q_jump),x(params.pos_quat).');
        % err = q_jump - x(params.pos_quat).';
        % xp(params.pos_quat(1)) = x(params.pos_quat(1)) + params.gamma(1)*(err(1)) + params.gamma(5)*(err(2)) + params.gamma(9)*(err(3)) + params.gamma(13)*(err(4));
        % xp(params.pos_quat(2)) = x(params.pos_quat(2)) + params.gamma(2)*(err(1)) + params.gamma(6)*(err(2)) + params.gamma(10)*(err(3))+ params.gamma(14)*(err(4));
        % xp(params.pos_quat(3)) = x(params.pos_quat(3)) + params.gamma(3)*(err(1)) + params.gamma(7)*(err(2)) + params.gamma(11)*(err(3))+ params.gamma(15)*(err(4));
        % xp(params.pos_quat(4)) = x(params.pos_quat(4)) + params.gamma(4)*(err(1)) + params.gamma(8)*(err(2)) + params.gamma(12)*(err(3))+ params.gamma(16)*(err(4));
        % xp(params.pos_quat(1)) = x(params.pos_quat(1)) + params.gamma(1)*(err(1));
        % xp(params.pos_quat(2)) = x(params.pos_quat(2)) + params.gamma(1)*(err(2));
        % xp(params.pos_quat(3)) = x(params.pos_quat(3)) + params.gamma(1)*(err(3));
        % xp(params.pos_quat(4)) = x(params.pos_quat(4)) + params.gamma(1)*(err(4));
%         xp(params.pos_quat(1)) = (1-params.gamma(1))*x(params.pos_quat(1)) + params.gamma(1)*q_jump(1);
%         xp(params.pos_quat(2)) = (1-params.gamma(1))*x(params.pos_quat(2)) + params.gamma(1)*q_jump(2);
%         xp(params.pos_quat(3)) = (1-params.gamma(1))*x(params.pos_quat(3)) + params.gamma(1)*q_jump(3);
%         xp(params.pos_quat(4)) = (1-params.gamma(1))*x(params.pos_quat(4)) + params.gamma(1)*q_jump(4);

        % bias
        % xp(params.pos_bias_w(1)) = x(params.pos_bias_w(1)) + params.gamma(2)*delta(1) + params.gamma(3)*delta(2) + params.gamma(4)*delta(3);
        % xp(params.pos_bias_w(2)) = x(params.pos_bias_w(2)) + params.gamma(5)*delta(1) + params.gamma(6)*delta(2) + params.gamma(7)*delta(3);
        % xp(params.pos_bias_w(3)) = x(params.pos_bias_w(3)) + params.gamma(8)*delta(1) + params.gamma(9)*delta(2) + params.gamma(10)*delta(3);
        % xp(params.pos_bias_w(1)) = x(params.pos_bias_w(1)) + 0*(params.gamma(2)*err(1) + params.gamma(3)*err(2) + params.gamma(4)*err(3) + params.gamma(5)*err(4));
        % xp(params.pos_bias_w(2)) = x(params.pos_bias_w(2)) + 0*(params.gamma(6)*err(1) + params.gamma(7)*err(2) + params.gamma(8)*err(3) + params.gamma(9)*err(4));
        % xp(params.pos_bias_w(3)) = x(params.pos_bias_w(3)) + 0*(params.gamma(10)*err(1) + params.gamma(11)*err(2) + params.gamma(12)*err(3) + params.gamma(13)*err(4));

        % normalize quaternion
        xp(params.pos_quat) = quatnormalize(xp(params.pos_quat).');

        % stability analysis - quaternion
        if 0
            GAMMA = diag([params.gamma(1), params.gamma(2), params.gamma(3)]);
            q = xp(params.pos_quat)';
            q = [q(2:4)'; q(1)];
            w = xp(params.pos_w)';
            Aq = double(params.fAqbar(q(1),q(2),q(3),q(4),w(1),w(2),w(3)));
            AqEXP = expm(Aq*params.Ts*params.UWB_samp);
    %         A2Q = double(params.fA2Qbar(rollhat,pitchhat,yawhat));
            A2Q = double(params.fA2Qbar(0,0,0));
            Q2A = double(params.fQ2Abar(q(1),q(2),q(3),q(4)));
            A2Qtrue = quatnormalize(double(params.fA2Q(0,0,0))');
            AqJump = (eye(4) - A2Q*GAMMA*Q2A);
    %         MONOTROMIC = AqJump*AqEXP(1:4,1:4);
            MONOTROMIC = AqJump;
            obs.init.params.EIGQUAT(:,pos(1)) = sort(real(eig(MONOTROMIC(1:3,1:3))));
    
            % Lyapunov function
            q_j = obs.init.X(obs.init.traj).val(params.pos_quat,max(1,pos(1)-obs.init.params.UWB_samp));
            qhat_j = obs.init.X_est(obs.init.traj).val(params.pos_quat,max(1,pos(1)-obs.init.params.UWB_samp));
            q_i = obs.init.X(obs.init.traj).val(params.pos_quat,max(1,pos(1)));
            qhat_i = x(params.pos_quat);
            qhat_i_jump = xp(params.pos_quat);
    
            % standard difference
            e_j = q_j - qhat_j; % error after last jump
            e_i = q_i - qhat_i; % error before this jump
            e_i_jump = q_i - qhat_i_jump; % error after this jump
            Ve_j = norm(e_j)^2;
            Ve_i = norm(e_i)^2;
            Ve_i_jump = norm(e_i_jump)^2;
    
            % quat product
    %         e_j = quatmultiply(q_j',quatinv(qhat_j')); % error after last jump
    %         e_i = quatmultiply(q_i',quatinv(qhat_i')); % error before this jump
    %         e_i_jump = quatmultiply(q_i',quatinv(qhat_i_jump')); % error after this jump
    %         Ve_j = quatnorm(e_j)^2;
    %         Ve_i = quatnorm(e_i)^2;
    %         Ve_i_jump = quatnorm(e_i_jump)^2;
    
            obs.init.params.INCREASE(pos(1)) = abs(Ve_i - Ve_j);
            obs.init.params.DECREASE(pos(1)) = abs(Ve_i_jump - Ve_i);
        end

        
        
        x = xp;               
    end   

    % Skew matrix - eq. 39 Challa
    q = x(params.pos_quat);
    W = x(params.pos_w) - x(params.pos_bias_w);
    S = [0      -W(3)   +W(2); ...
         +W(3)  0       -W(1); ...
         -W(2)  +W(1)   0];
    OMEGA = [+S     W; ...
             -W'    0];

    % measure
    w = reshape(y(params.pos_w_out),params.rotation_dim,1);

    % quaternion dynamics - eq. 40 armesto
    x_dot(params.pos_quat) = 0.5*OMEGA*q;

    % model dynamics - angular velocity - eq. 41b armesto continuous
    x_dot(params.pos_w) = params.Asfer(end,end)*x(params.pos_w) + params.Bsfer(end)*w;        



    
end