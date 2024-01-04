%% measure_general
% file: measure_general.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function implements the output mapping of a general
%              state space system
% INPUT:
%           x: state vector
%           Params: structure with all the necessary parameter to the model
%           t: time instant (could be used - see ref)
%           u: control input (could be used - see ref)
%           obs: obsopt instance. It is added in case you need to store
%                something out of the function scope
% OUTPUT:
%           y: output measurement
%           obs: updated obsopt instance
function [y, obs] = measure_general(x,Params,t,u,obs)

    % get the observed components of the state vector from the
    % Params.ObservedState vector
    y = x(Params.ObservedState,:);
    
end