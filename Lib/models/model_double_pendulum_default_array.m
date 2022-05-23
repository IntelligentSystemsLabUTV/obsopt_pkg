%% double pendlum model
function x_dot = model_double_pendulum_default_array(t, x, params, obj)

    x_dot = zeros(length(x),1);
    
    
    L1 = params.Lt1;
    L2 = params.Lt2;
    M1 = params.M1;
    M2 = params.M2;
    C1 = params.c1;
    C2 = params.c2;
    g = params.g;
    
    x_dot(1) = x(3);
    x_dot(2) = x(4);
    
    delta = M2/(M1+M2);
    
    % input
    tdiff = params.time-t;
    pos = find(tdiff == min(tdiff));
    drive = obj.drive(obj.init.X(obj.init.traj).val(:,pos),x);
    tau = obj.setup.params.input(t,drive,obj.init.params);
     
    x_dot(3) = (-C1*x(3) + delta*tau(1)/(M2*L1) - delta*L2*x(4)^2*sin(x(1)-x(2)) - g*cos(x(1)) - delta*cos(x(1)-x(2))*(tau(2)/(M2*L2) + L1*x(3)^2*sin(x(1)-x(2) - g*cos(x(2)))))/(L1*(1-delta*cos(x(1)-x(2))^2));
    
    x_dot(4) = (-C2*x(4) + tau(2)/(M2*L2) + L1*x(3)^2*sin(x(1)-x(2)) - g*cos(x(2)) - cos(x(1)-x(2))*(delta*tau(1)/(M2*L1) + delta*L2*x(4)^2*sin(x(1)-x(2) - g*cos(x(1)))))/(L2*(1-delta*cos(x(1)-x(2))^2));
    
    % params dynamics
    x_dot(5) = 0.05 + 0.05*randn(1);    % random walk
    x_dot(6) = 0.05 + 0.05*randn(1);    % random walk
end