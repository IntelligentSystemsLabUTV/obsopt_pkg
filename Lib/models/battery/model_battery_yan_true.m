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
function x_dot = model_battery_yan_true(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);    
    
    % compute the control
    params.u = params.input(t,x,params);
    
    % model dynamics - discrete
    % Zk (SOC)
    x_dot(1) = x(1,:) -params.u(1,:)./params.Cn.*obs.setup.Ts;
    % V1 (voltage RC)
    x_dot(2) = x(2,:)*exp(-obs.setup.Ts./(x(5,:).*x(6,:))) + params.u(1,:).*x(5,:).*(1-exp(-obs.setup.Ts./(x(5,:).*x(6,:))));
    
    % constant parameters
%     x_dot(3) = x(3,:);
%     x_dot(4) = x(4,:);
%     x_dot(5) = x(5,:);
%     x_dot(6) = x(6,:);
    
    % params dynamics (under development)
%     x_dot(3) = x(3,:) + 0.01*x(1,:);
%     x_dot(4) = x(4,:) + 0.01*x(1,:);
%     x_dot(5) = x(5,:) + 0.01*x(1,:);
%     x_dot(6) = x(6,:) + 0.01*x(1,:);

    x_dot(3) = x(3,:) + x(7,:);
    x_dot(4) = x(4,:) + x(8,:);
    x_dot(5) = x(5,:) + x(9,:);
    x_dot(6) = x(6,:) + x(10,:);   
    x_dot(7) = x(7,:);
    x_dot(8) = x(8,:);
    x_dot(9) = x(9,:);
    x_dot(10) = x(10,:); 

%     x_dot(3) = x(3,:) + 0.02*randn + 0.01;
%     x_dot(4) = x(4,:) + 0.02*randn + 0.01;
%     x_dot(5) = x(5,:) + 0.02*randn + 0.01;
%     x_dot(6) = x(6,:) + 0.02*randn + 0.01;
end