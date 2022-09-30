function X = testFcn(t,x,u,Ts,data,x0)

    % define the time span of the integration
    tspan = 0:Ts;   

    % set initial condition
    if t==0
        x = x0;
    end

    % true system - correct initial condition and no noise
    % considered
    X = odeDD(@(t,x)model_battery_tushar_reference_SIM(t, x, u, Ts, data), tspan, x); 
    X = X.y(:,end);

end