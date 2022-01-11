%% double pendlum model
function x_dot = model_double_pendulum_input(t, x, params)

    x_dot = zeros(length(x),1);
    
    params = params_update(params,x);
    
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
    params.u = params.input(x,params);
    tau = params.u;
     

    x_dot(3) = (-C1*x(3) + delta*tau(1)/(M2*L1) - delta*L2*x(4)^2*sin(x(1)-x(2)) - g*cos(x(1)) - delta*cos(x(1)-x(2))*(tau(2)/(M2*L2) + L1*x(3)^2*sin(x(1)-x(2) - g*cos(x(2)))))/(L1*(1-delta*cos(x(1)-x(2))^2));
    
    x_dot(4) = (-C2*x(4) + tau(2)/(M2*L2) + L1*x(3)^2*sin(x(1)-x(2)) - g*cos(x(2)) - cos(x(1)-x(2))*(delta*tau(1)/(M2*L1) + delta*L2*x(4)^2*sin(x(1)-x(2) - g*cos(x(1)))))/(L2*(1-delta*cos(x(1)-x(2))^2));
    
    x_dot(5:end) = 0;

end