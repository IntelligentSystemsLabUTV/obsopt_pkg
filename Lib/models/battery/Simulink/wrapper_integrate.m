%%
function y = wrapper_integrate(t,x,u,Ts,data,Time)

    % define the time span of the integration
    tspan = 0:Ts;     

    % true system - correct initial condition and no noise
    % considered
    X = odeDD(@(t,x)model_battery_tushar_reference_SIM(t, x, u, Ts, data, Time), tspan, x); 
%     X = odeDD(@(t,x)easymodel(t, x, data, Time), tspan, x);
 
    y = X.y(:,end);

end

function y = easymodel(t,x,data,Time)

    y = zeros(size(x));

    % get time index
    tdiff = Time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first'); 
    pos = min(pos,length(Time));

    y(1) = data(1,1,pos);

end