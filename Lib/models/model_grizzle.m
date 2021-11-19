%% model for pendulum
function x_dot = model_grizzle(t,x,params)

    x_dot = zeros(length(x),1);
    
    S = params.i-params.a*x(1)-params.b*x(2);
    
    x_dot(1) = params.c*S/(params.d+S)*x(1)-params.u(1)*x(1);
    x_dot(2) = params.e*S/((params.f+S)*(params.g+x(3)))*x(2)-params.u(1)*x(2);
    x_dot(3) = -params.h*x(1)*x(3)-params.u(1)*x(3)+params.u(1)*params.u(2);
end