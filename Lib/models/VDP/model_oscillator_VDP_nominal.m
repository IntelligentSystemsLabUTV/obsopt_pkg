%% MODEL_OSCILLATOR_VDP
% file: model_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the dynamics equation of a Van der 
% Pol oscillator
% INPUT:
% t: time instant
% x: state vector
% Params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function [x_dot, x] = model_oscillator_VDP_nominal(tspan,x,Params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.T-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    u = obs.Functions.Input(tspan,x,Params);
    obs.U(obs.Params.Traj).val(:,pos) = u;
    
    % model dynamics
    x_dot(1) = Params.eps*(x(2) + u(1));
    x_dot(2) = Params.eps*(-x(1) + x(3)*(1-x(1)^2)*x(2) + u(2));
    
    % Params dynamics (under development)
    x_dot(3) = Params.A_mu*cos(Params.F_mu*tspan(1)+Params.Phi_mu);    % oscillates
end