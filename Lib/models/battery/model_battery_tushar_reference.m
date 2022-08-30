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
function x_dot = model_battery_tushar_reference(t,x,params,obs)

    % init the dynamics 
    x_dot = x;    
    % compute the control
    params.u = params.input(t,x,params);
    
    % save input
    tdiff = obs.setup.time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first'); 
    pos = min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3));
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;

    %%% model equations %%%
    tau_1 = x(5,:).*x(6,:);
    a1 = exp(-obs.setup.Ts/tau_1);
    b1 = x(5,:).*(1 - exp(-obs.setup.Ts/tau_1));
    
    % model dynamics - discrete
    % Zk (SOC)
    x_dot(1,:) = x(1,:) - params.eta * obs.setup.Ts *params.u(1,:)/params.C_n;

    % V1 (voltage RC)
    x_dot(2,:) = a1*x(2,:) + b1*params.u(1,:);
    
    %%%% params update %%%%
    % all the other states depending on SOC. There is no dynamics here,
    % just update, as we are considering a DT model. 
    
    %     ocv = [3.5042   3.5573    3.6009    3.6393    3.6742    3.7121    3.7937    3.8753    3.9700    4.0764    4.1924];
    %     soc = [0   10  20  30  40  50  60  70  80  90  100];
    %     R0 = [0.0114    0.0110    0.0113    0.0112    0.0110    0.0107    0.0107    0.0107    0.0109    0.0113    0.0116];
    %     R1 = [0.0103    0.0067    0.0051    0.0043    0.0039    0.0034    0.0033    0.0033    0.0033    0.0033    0.0028];
    %     C1 = [0.2288    0.6122    1.8460    2.0975    1.5254    1.0440    1.3903    1.6694    1.5784    1.2165    0.9118];

    %     I = params.input(t,x,params);
    %     Vt = params.measure(x,params,t);
    %     Voc_temp = Vt + x(2) + (I* params.R0);
    %     soc_temp = spline(ocv,soc,params_out.Voc); 
    
    % params dynamics - depending on x(1) = SOC
    x_dot(3,:) = spline(params.input_soc, params.input_OCV, x(1,:));
    x_dot(4,:) = spline(params.input_soc, params.input_R0, x(1,:));
    x_dot(5,:) = spline(params.input_soc, params.input_R1, x(1,:));
    x_dot(6,:) = spline(params.input_soc, params.input_C1, x(1,:));
    
%     x_dot(3,:) = 0*x(3,:) + params.alpha_Voc + params.beta_Voc*x(1,:) + params.gamma_Voc*(x(1,:).^2);
%     x_dot(4,:) = 0*x(4,:) + params.alpha_R0 + params.beta_R0*x(1,:) + params.gamma_R0*(x(1,:).^2);
%     x_dot(5,:) = 0*x(5,:) + params.alpha_R1 + params.beta_R1*x(1,:) + params.gamma_R1*(x(1,:).^2);
%     x_dot(6,:) = 0*x(6,:) + params.alpha_C1 + params.beta_C1*x(1,:) + params.gamma_C1*(x(1,:).^2);
    
    

end