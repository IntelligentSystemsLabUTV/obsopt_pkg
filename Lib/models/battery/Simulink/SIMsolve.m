%%
function y = SIMsolve(t,x,u,Ts,data)

    % define the time span of the integration
    tspan = 0:Ts;   

    % true system - correct initial condition and no noise
    % considered
%     X = odeDD(@(t,x)model_battery_tushar_reference_SIM(t, x, u, Ts, data), tspan, x); 
    X = odeDD(@(t,x)sin(x), tspan, x);
    y = X.y(:,end);

end