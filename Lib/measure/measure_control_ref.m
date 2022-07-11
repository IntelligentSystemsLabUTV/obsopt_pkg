%% MODEL_REFERENCE
% file: measure_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function implements the output mapping of a general
% state space system
% INPUT:
% x: state vector
% params: structure with all the necessary parameters 
% t: time instant (may be not used)
% OUTPUT:
% y: output measurement
function y = measure_control_ref(x,params,t)

    % LTI system - C matrix
    C = [0, 0, 0, 0, 1];
    
    % get the observed components of the state vector
    y(1:2,:) = x(1:2,:);
    y(3,:) = 0*C*x(1:5,:);

end