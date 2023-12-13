%% Runaway electrons model
% x = [T W]'
function [x_dot, x] = model_runaway(tspan, x, params, obs)

    % init the dynamics
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end  

        drive = x;
    params.u = obs.setup.params.input(tspan,drive,obs.init.params);
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;
    obs.init.input_story_ref(obs.init.traj).val(:,pos) = params.u;
    
    % just for reading ease
    T = x(1);
    W = x(2);

    x_dot(1) = params.eps_coef*(-2*T*W - 2*params.S + params.Q);
    x_dot(2) = params.eps_coef*(-params.ni*W + params.gamma*(T*W + params.S) - params.gamma1*W/(1+W/params.Wt));
end