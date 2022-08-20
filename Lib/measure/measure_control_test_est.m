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
function y = measure_control_test_est(x,params,t,u)

    % LTI system - C matrix
    C = [params.c0est params.c1est];
    
    % get the observed components of the state vector        
    y(1,:) = 1*C(1:2)*x(6:7,:) + params.d0est*u(4,:);        % yhat (for plant)
    y(2,:) = 1*C(1:2)*x(1:2,:) + params.d0est*u(1,:);        % yhat (for reference)
    
end