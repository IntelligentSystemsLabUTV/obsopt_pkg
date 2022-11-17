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
function y = measure_TCV_reference(x,params,t,u)
    
    
    % get the observed components of the state vector 
    range = params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+1:params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+params.n;
    y = params.C*x(range,:) + params.D*u;
    
end