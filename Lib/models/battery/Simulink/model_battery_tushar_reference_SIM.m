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
function x_dot = model_battery_tushar_reference_SIM(t,x,u,Ts,data,Time)

    % init the dynamics 
    x_dot = x;      

    % get time index
    tdiff = Time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first'); 
    pos = min(pos,length(Time));

    % get params
    C_n = data(1,1,pos);
    eta = data(2,1,pos);
    input_soc = data(3:5,1,pos);
    input_OCV = data(6:8,1,pos);
    input_R0 = data(9:11,1,pos);
    input_R1 = data(12:14,1,pos);
    input_C1 = data(15:17,1,pos);

    %%% model equations %%%
    tau_1 = x(5,:).*x(6,:);
    a1 = exp(-Ts/tau_1);
    b1 = x(5,:).*(1 - exp(-Ts/tau_1));
    
    % model dynamics - discrete
    % Zk (SOC)
    x_dot(1,:) = x(1,:) - eta * Ts *u/C_n;

    % V1 (voltage RC)
    x_dot(2,:) = a1*x(2,:) + b1*u;
    
    %%%% params update %%%%    
    % params dynamics - depending on x(1) = SOC
%     x_dot(3,:) = spline(input_soc, input_OCV, x(1,:));
%     x_dot(4,:) = spline(input_soc, input_R0, x(1,:));
%     x_dot(5,:) = spline(input_soc, input_R1, x(1,:));
%     x_dot(6,:) = spline(input_soc, input_C1, x(1,:));
    
    x_dot(3,:) = 0*x(3,:) + x(7,:) + x(11,:).*x(1,:) + x(15,:).*(x(1,:).^2);
    x_dot(4,:) = 0*x(4,:) + x(8,:) + x(12,:).*x(1,:) + x(16,:).*(x(1,:).^2);
    x_dot(5,:) = 0*x(5,:) + x(9,:) + x(13,:).*x(1,:) + x(17,:).*(x(1,:).^2);
    x_dot(6,:) = 0*x(6,:) + x(10,:) + x(14,:).*x(1,:) + x(18,:).*(x(1,:).^2);
    
    

end