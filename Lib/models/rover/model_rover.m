% MODEL_ROVER
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
    params.u = obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos(1)));
    obs.init.input_story(obs.init.traj).val(:,pos(1)) = params.u(:,1);   

    %%%%%%%%%%% HYBRID OBSERVER MODEL %%%%%%%%%%%%%

    if (params.hyb && ~params.EKF) && ~params.dryrun

        % meas available
        y = obs.init.Y_full_story(obs.init.traj).val(1,:,pos(1));
        a = y(params.pos_acc_out);     
        xp = x;
    
        if params.sferlazza == 0
            % Jump - only on the UWB
            if (mod(pos(1),params.UWB_samp) == 0) && (~params.EKF)
        
                % adjacency matrix
                for dim=1:params.space_dim
                    Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
                end
        
                %%% TEST %%%
                p_jump = obs.init.params.p_jump(obs.init.traj).val(:,pos(1)/params.UWB_samp);
                p_jump_der = obs.init.params.p_jump_der(obs.init.traj).val(:,pos(1)/params.UWB_samp);            
                
                % jump map - x
                xp(1) = x(1) + params.theta(1)*(p_jump(1)-x(1)) + params.theta(4)*(p_jump(1)-x(1))^3;
                xp(2) = x(2) + params.theta(2)*(p_jump(1)-x(1)) + params.theta(5)*(p_jump(1)-x(1))^3;
                xp(3) = x(3) + params.theta(3)*(p_jump(1)-x(1)) + params.theta(6)*(p_jump(1)-x(1))^3;
                xp(4) = x(4);
                        
                % jump map - y
                xp(5) = x(5) + params.theta(1)*(p_jump(2)-x(5)) + params.theta(4)*(p_jump(2)-x(5))^3;
                xp(6) = x(6) + params.theta(2)*(p_jump(2)-x(5)) + params.theta(5)*(p_jump(2)-x(5))^3;
                xp(7) = x(7) + params.theta(3)*(p_jump(2)-x(5)) + params.theta(6)*(p_jump(2)-x(5))^3;
                xp(8) = x(8);               
    
                % jump map - z
                xp(9) = x(9) + params.theta(1)*(p_jump(3)-x(9)) + params.theta(4)*(p_jump(3)-x(9))^3;
                xp(10) = x(10) + params.theta(2)*(p_jump(3)-x(9)) + params.theta(5)*(p_jump(3)-x(9))^3;
                xp(11) = x(11) + params.theta(3)*(p_jump(3)-x(9)) + params.theta(6)*(p_jump(3)-x(9))^3;
                xp(12) = x(12);   

                x = xp;
            end   
            
        else

            % Jump - only on the UWB
            if (mod(pos(1),params.UWB_samp) == 0) && (~params.EKF)
        
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

    %%%%%%%%%%%%% EKF MODEL %%%%%%%%%%%%
    elseif (params.EKF && ~params.hyb) && ~params.dryrun

        if 1
            % model dynamics
            % x axis
            x_dot(1) = x(2);    
            x_dot(2) = x(4);            
            
            % y axis
            x_dot(5) = x(6);    
            x_dot(6) = x(8);               
    
            % z axis
            x_dot(9) = x(10);    
            x_dot(10) = x(12);               
        else
            % perfect realization
            A = params.ssd_EKF.A;
            B = params.ssd_EKF.B;
            C = params.ssd_EKF.C;
            D = params.ssd_EKF.D;

            for i=1:params.space_dim
                range = [params.pos_p(i) params.pos_v(i)];
                evol = A*x(range) + B*x(params.pos_acc(i));                
                x_dot(range) = (evol - x(range))/params.Ts;
            end
            
        end

    elseif params.dryrun

    %%%%%%%%%%%%% REFERENCE MODEL %%%%%%%%%
        x_dot = obs.setup.model_reference(tspan,x,params,obs);

    else

    %%%%%%%%%%%%% ERROR %%%%%%%%%
        error('what do you wanna integrate mate?')

    end


    
end