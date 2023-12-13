%% MODEL_ROVER
% file: model_rover.m
% author: Federico Oliva
% date: 05/12/2022
% description: this function describes the dynamics equation of a rover
% t: time instant
% x: state vector
% y: measures (filtred?)
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_observer(tspan,x,y,params,obs)

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
           
    alpha = [-0.5 -0.5 -0.2 -0.2];     
    % observer dynamics simplified
    x_dot(1) =  x(3) + alpha(1)*(x(1)-x(5));
    x_dot(2) =  x(4) + alpha(2)*(x(2)-x(6));    
    x_dot(3) = alpha(3)*x(3) + y(params.pos_acc(1));
    x_dot(4) = alpha(4)*x(4) + y(params.pos_acc(2));
    x_dot(5) = 0;
    x_dot(6) = 0;
    
    
end