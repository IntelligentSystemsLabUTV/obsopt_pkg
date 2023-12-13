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
function x_dot = model_battery_tushar_reference_CT(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(size(x));    
    % compute the control
    params.u = params.input(t,x,params);
    
    % save input
    tdiff = obs.setup.time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first'); 
    pos = min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3));
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;

    %%% model equations %%%
    tau_1 = x(5,:).*x(6,:);    
    
    % model dynamics - discrete
    % Zk (SOC)
    x_dot(1,:) = -params.u(1,:)/params.C_n;

    % V1 (voltage RC)
    x_dot(2,:) = -x(2,:)/tau_1 + params.u(1,:)/x(6,:);
    
    %%%% params update %%%%    
    % params dynamics - depending on x(1) = SOC    
    x_dot(3,:) = params.alpha_Voc;
    x_dot(4,:) = params.alpha_R0;
    x_dot(5,:) = params.alpha_R1;
    x_dot(6,:) = params.alpha_C1;
    

end