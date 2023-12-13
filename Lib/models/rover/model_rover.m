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
    
        % Jump - only on the UWB
        if (mod(pos(1),params.UWB_samp) == 0) && (~params.EKF)
    
            % adjacency matrix
            for dim=1:params.space_dim
                Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
            end
    
            %%% TEST %%%
            p_jump = obs.init.params.p_jump(obs.init.traj).val(:,pos(1)/params.UWB_samp);
            p_jump_der = obs.init.params.p_jump_der(obs.init.traj).val(:,pos(1)/params.UWB_samp);

            % gamma
            normx = norm(x(params.pos_p(1)));
            normy = norm(x(params.pos_p(2)));
            normz = norm(x(params.pos_p(3)));
            gamma(1) = params.theta(1) + 0*params.theta(2)*normx;
            gamma(2) = params.theta(1) + 0*params.theta(2)*normy;
            gamma(3) = params.theta(1) + 0*params.theta(2)*normz;
            
            % jump map - x
            x(1) = gamma(1)*x(1) + (1-gamma(1))*p_jump(1);
%             x(2) = params.theta(3)*x(2) + (1-params.theta(3))*p_jump_der(1);
%             x(3) = params.theta(4)*x(3);
%             x(4) = params.theta(5)*x(4);
    
            % jump map - y
            x(6) = gamma(2)*x(6) + (1-gamma(2))*p_jump(2);
%             x(7) = params.theta(3)*x(7) + (1-params.theta(3))*p_jump_der(2);
%             x(8) = params.theta(4)*x(8);
%             x(9) = params.theta(5)*x(9);

            % jump map - z
            x(11) = gamma(3)*x(11) + (1-gamma(3))*p_jump(3);
%             x(12) = params.theta(3)*x(12) + (1-params.theta(3))*p_jump_der(3);
%             x(13) = params.theta(4)*x(13);
%             x(14) = params.theta(5)*x(14);
        end
    
        %%%% OBSERVER DYNAMICS %%%
        % model dynamics
        % x axis
        x_dot(1) = x(2);
        x_dot(2) = x(3) - params.alpha(1)*x(2) + 1*params.alpha(2)*abs(x(3))*x(3);
        x_dot(3) = x(4) + params.beta(1)*a(1);
        x_dot(4) = -params.C(1)*x(3) -params.C(2)*x(4) + params.beta(2)*a(1);
    
        % y axis
        x_dot(6) = x(7);
        x_dot(7) = x(8) - params.alpha(1)*x(7) + 1*params.alpha(2)*abs(x(8))*x(8);
        x_dot(8) = x(9) + params.beta(1)*a(2);
        x_dot(9) = -params.C(1)*x(8) -params.C(2)*x(9) + params.beta(2)*a(2);  

        % z axis
        x_dot(11) = x(12);
        x_dot(12) = x(13) - params.alpha(1)*x(12) + 1*params.alpha(2)*abs(x(13))*x(13);
        x_dot(13) = x(14) + params.beta(1)*a(3);
        x_dot(14) = -params.C(1)*x(13) -params.C(2)*x(14) + params.beta(2)*a(3);  

    %%%%%%%%%%%%% EKF MODEL %%%%%%%%%%%%
    elseif (params.EKF && ~params.hyb) && ~params.dryrun

        % model dynamics
        % x axis
        x_dot(1) = x(2);    
        x_dot(2) = x(3);
        x_dot(3) = 0;
        
        % y axis
        x_dot(6) = x(7);    
        x_dot(7) = x(8);   
        x_dot(8) = 0;

        % z axis
        x_dot(11) = x(12);    
        x_dot(12) = x(13);   
        x_dot(13) = 0;

    elseif params.dryrun

    %%%%%%%%%%%%% REFERENCE MODEL %%%%%%%%%
        x_dot = obs.setup.model_reference(tspan,x,params,obs);

    else

    %%%%%%%%%%%%% ERROR %%%%%%%%%
        error('what do you wanna integrate mate?')

    end


    
end