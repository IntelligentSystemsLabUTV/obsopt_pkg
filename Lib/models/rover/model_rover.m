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
function x_dot = model_rover(tspan,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    params.u = params.input(tspan,x,params);
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;    

    % meas available
    y = obs.init.Y_full_story(obs.init.traj).val(1,:,pos(1));
    a = y(params.pos_acc);
    D_meas = y(params.pos_dist);

    % Jump
    if mod(pos(1),params.UWB_samp) == 0 
        % adjacency matrix
        Pa(1,:) = x(params.pos_anchor(1):2:params.pos_anchor(end));
        Pa(2,:) = x(params.pos_anchor(2):2:params.pos_anchor(end));
        p_jump = uwb_est_v2(x(params.pos_p),Pa,D_meas,obs.setup.params);

        % jump map
        x(5) = x(1);
        x(6) = p_jump(1);        
        x(11) = x(7);
        x(12) = p_jump(2);
    end
    
    % model dynamics
    % x axis
    x_dot(1) = x(2) + params.L(1)*params.G(1)*(x(6)-x(5));
    x_dot(2) = x(3) -params.alpha*x(2) + params.L(1)^2*params.G(2)*(x(6)-x(5)) + params.u(1);
    x_dot(3) = x(4) + params.K(1)*params.C(1)*(a(1)-x(3));
    x_dot(4) = params.K(1)^2*params.C(2)*(a(1)-x(3));
    x_dot(5) = 0;
    x_dot(6) = 0;
    
    % y axis
    x_dot(7) = x(8) + params.L(2)*params.G(3)*(x(11)-x(12));
    x_dot(8) = x(9) - params.alpha*x(8) + params.L(2)^2*params.G(4)*(x(11)-x(12)) + params.u(2);
    x_dot(9) = x(10) + params.K(2)*params.C(3)*(a(2)-x(9));
    x_dot(10) = params.K(2)^2*params.C(4)*(a(2)-x(9));
    x_dot(11) = 0;
    x_dot(12) = 0;

    % all the remaining are the anchors
    
end