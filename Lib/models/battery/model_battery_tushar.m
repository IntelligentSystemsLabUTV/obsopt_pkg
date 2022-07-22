%% MODEL_BATTERY
% file: model_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function describes the dynamics equation of a Van der 
% Pol oscillator
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_battery_tushar(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);    
    % compute the control
    params.u = params.input(t,x,params);

    tau_1 = params.R1 * params.C1;
    a1 = exp(-obs.setup.Ts/tau_1);
    b1 = params.R1 * (1 - exp(-obs.setup.Ts/tau_1));
    
    % model dynamics - discrete
    % Zk (SOC)
    x_dot(1) = x(1,:) - params.eta * obs.setup.Ts *params.u(1,:)/params.C_n;

    % V1 (voltage RC)
    x_dot(2) = a1* x(2,:) + b1* params.u(1,:);
    

end