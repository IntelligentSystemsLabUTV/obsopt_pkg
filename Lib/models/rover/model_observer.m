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
     
    % observer dynamics simplified
    x_dot(1) = x(3);
    x_dot(2) = x(4);
    
    x_dot(3) = y(params.pos_acc(1));
    x_dot(4) = y(params.pos_acc(2));
    %x_dot(3) = params.u(1)/params.m;
    %x_dot(4) = params.u(2)/params.m;
    
    
end