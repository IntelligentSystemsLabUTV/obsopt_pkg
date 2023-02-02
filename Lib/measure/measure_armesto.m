%% MODEL_REFERENCE
% file: measure_general.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements the output mapping of a general
% state space system
% INPUT:
% x: state vector
% params: structure with all the necessary parameters 
% t: time instant (may be not used)
% OUTPUT:
% y: output measurement
function [y, obs] = measure_armesto(x,params,t,u,obs)

    % compute the time index
    pos = zeros(1,length(t));
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end

    % get the observed components of the state vector
    y = x(params.observed_state,:);    

    % add bias
    y(params.pos_biased_out) = y(params.pos_biased_out) + x(params.pos_bias);
    
end