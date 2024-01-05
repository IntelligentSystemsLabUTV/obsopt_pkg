    %% model_oscillator_VDP
% file: model_oscillator_VDP.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function describes the dynamics equation of a Van der 
%              Pol oscillator
% INPUT:
%           tspan: time interval
%           x: state vector
%           Params: structure with all the necessary parameters 
%           obs: obsopt class instance (could be used)
% OUTPUT:
%           x_dot: state derivative - continuous
%           x_plus: state reset - discrete (see ref)
function [x_dot, x_plus] = model_oscillator_VDP(tspan,x,Params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index. here we find the index in the time vector
    % corresponding to the elements of the time interval. This is done
    % to retrieve inputs or other stuff from the storage in obs

    % init the position index vector
    pos = zeros(length(tspan));

    % cycle over the time interval
    for i=1:length(tspan)

        % compute the difference with each element of the time vector
        tdiff = obs.T - tspan(i);   

        % find the index in which the difference is minimum. that is the
        % corresonding time index
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');   

        % safe check, we don't want index zero
        pos(i) = max(1,pos(i));      

    end    
    
    % compute the control
    u = obs.Functions.Input(tspan,x,Params);
    % store the input 
    obs.U(obs.Params.Traj).val(:,pos) = u;

    % continuous dynamics
    
    % model dynamics (see ref or the related paper)
    x_dot(1) = Params.eps*(x(2) + u(1));
    x_dot(2) = Params.eps*(-x(1) + x(3)*(1-x(1)^2)*x(2) + u(2));
    
    % Params dynamics. Normally is zero because parameters are constant.
    % Might not always be the case (time-variant systems)
    x_dot(3) = 0; 

    % discrete dynamics (see ref)
    x_plus = x;
    
end