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
function x_dot = model_battery_tushar_CT(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(size(x));
    
    % compute the control
    params.u = 1*params.input(t,x,params);        

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
    x_dot(3,:) = params.alpha_Voc + params.beta_Voc*x(1,:) + params.gamma_Voc*(x(1,:).^2) + params.delta_Voc*(x(1,:).^3) + params.eps_Voc*(x(1,:).^4) + params.xi_Voc*(x(1,:).^5);
    x_dot(4,:) = params.alpha_R0 + params.beta_R0*x(1,:) + params.gamma_R0*(x(1,:).^2) + params.delta_R0*(x(1,:).^3) + params.eps_R0*(x(1,:).^4) + params.xi_R0*(x(1,:).^5);
    x_dot(5,:) = params.alpha_R1 + params.beta_R1*x(1,:) + params.gamma_R1*(x(1,:).^2) + params.delta_R1*(x(1,:).^3) + params.eps_R1*(x(1,:).^4) + params.xi_R1*(x(1,:).^5);
    x_dot(6,:) = params.alpha_C1 + params.beta_C1*x(1,:) + params.gamma_C1*(x(1,:).^2) + params.delta_C1*(x(1,:).^3) + params.eps_C1*(x(1,:).^4) + params.xi_C1*(x(1,:).^5);
    
    

end