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
function y = measure_control_test(x,params,t,u)

    % LTI system - C matrix
    C = [params.C1 params.C2];
    
    % get the observed components of the state vector        
    y(1,:) = 1*C(1:2)*x(6:7,:);        % yhat (for plant)
    y(2,:) = 1*C(1:2)*x(1:2,:);        % yhat (for reference)
    
end