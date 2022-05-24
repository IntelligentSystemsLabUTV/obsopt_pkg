%% battery model
function x_dot = model_battery(t, x, params, obj)

    x_dot = zeros(length(x),1);

    % control
    params.u = params.input(t,x,params);

    x_dot(1) = x(1) - params.u*params.eta*obj.setup.Ts/params.Q;
    x_dot(2) = exp(-obj.setup.Ts/(params.R1*params.C1)) + params.u*(1-exp(-obj.setup.Ts/(params.R1*params.C1)*params.R1));

    % params dynamics
%     x_dot(5) = 0.05 + 0.05*randn(1);    % random walk
%     x_dot(6) = 0.05 + 0.05*randn(1);    % random walk
end
