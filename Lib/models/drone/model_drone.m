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
function [x_dot, x] = model_drone(tspan,x,params,obs)
    
    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    params.u = obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos(1)));
    
    obs.init.input_story(obs.init.traj).val(:,pos(1)) = params.u(:,1);  
    y = obs.init.Y_full_story(obs.init.traj).val(1,:,pos(end))';

    % noise dynamics
%    x_dot(params.pos_bias_v) = 0*(x(params.pos_bias_v) + params.bias_v_enable*ones(3,1)*sin(tspan(1)));
    
    % switch gamma
    if tspan > 10
        tmp = params.gamma(1:3);
        params.gamma(1:3) = params.gamma(4:6);
        params.gamma(4:6) = tmp;
    end
    % 
    % if tspan > 100
    %     tmp = params.gamma(1:3);
    %     params.gamma(1:3) = params.gamma(4:6);
    %     params.gamma(4:6) = tmp;
    % end


    
    obs.init.params.gamma_story(round((tspan*100)+2),:) = params.gamma(1:3)';
    
    %%% model dynamics - translation    
    x_dot(params.pos_obs) = (1-(params.Ts*params.gamma(3)))*x(params.pos_p) + params.Ts*(params.gamma(1)*y(params.pos_uwb_out) + params.gamma(2)*y(params.pos_cam_out));

    x_dot(params.pos_p) = x(params.pos_v);
    x_dot(params.pos_v) = -params.u(1:3);



    % eq. 38 armesto
    %x_dot(params.pos_p) = (1-(params.Ts*params.gamma(3)))*x(params.pos_p) + params.Ts*(params.gamma(1)*y(params.pos_uwb_out) + params.gamma(2)*y(params.pos_cam_out));
    % eq. 37 armesto 
    %x_dot(params.pos_v) = (params.pos_v) + (params.Ts*x(params.pos_acc) + 0.5*params.Ts^2*x(params.pos_jerk)); %no dynamics, i just want the position
    % eq. 36 armesto
    %x_dot(params.pos_acc) = x(params.pos_acc) + (params.Ts*(x(params.pos_jerk) + params.u(1:3) + cross(x(params.pos_alpha),x(params.pos_v)) + cross(x(params.pos_omega),x(params.pos_acc))));
    % eq. 39 armesto
    %x_dot(params.pos_bias) = x(params.pos_bias) + (params.Ts*x(params.pos_bias_v));        

    %%% model dynamics - quaternion

    % Skew matrix - eq. 41 armesto
    % q = x(params.pos_quat);
    % S = [q(1) -q(2) -q(3) -q(4); ...
    %      q(2) +q(1) +q(4) -q(3);   ...
    %      q(3) -q(4) +q(1) +q(2);   ...
    %      q(4) +q(3) -q(2) +q(1)];
    % 
    % % quaternion dynamics - eq. 40 armesto
    % tmp = 0.5*params.Ts*x(params.pos_omega) + 0.25*params.Ts^2*x(params.pos_alpha);
    % den = x(params.pos_omega)+0.5*params.Ts*x(params.pos_alpha);
    % vec = [cos(norm(tmp)); ...
    %        sin(norm(tmp)/norm(den))*(den)];
    % if any(isnan(vec))
    %     x_dot(params.pos_quat) = [1 0 0 0]';
    % else
    %     x_dot(params.pos_quat) = S*vec;
    % end
    % x_dot(params.pos_quat) = x(params.pos_quat)' + 0*quatnormalize(x_dot(params.pos_quat)');
    % 
    % % model dynamics - angular velocity - eq. 41b armesto
    % x_dot(params.pos_omega) = x(params.pos_omega) + 0*params.Ts*(x(params.pos_alpha) + 0*params.u(4:6));

    % parameter dynamics
    x_dot(params.pos_gamma) = params.gamma; 
end