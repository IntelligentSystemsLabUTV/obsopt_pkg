%% model for pendulum
function x_dot = model_grizzle(t,x,params)

    x_dot = zeros(length(x),1);
    u = params.u;
    
    S = params.i-params.a*x(1)-params.b*x(2);
    
    x_dot(1) = params.c*S/(params.d+S)*x(1) + u(1);
    x_dot(2) = params.e*S/((params.f+S)*(params.g+x(3)))*x(2) +u(2);
    x_dot(3) = -params.h*x(1)*x(3) + u(3);
end