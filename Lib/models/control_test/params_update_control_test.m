%% PARAMS_UPDATE_OSCILLATOR_VDP
% file: params_update_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function updates the estimated parameters on an
% unstable LTI model
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_control_test(params,x)

    % assign params
    params_out = params;    

    % update model
    params_out.a0est = x(5);
    params_out.a1est = x(6);
    params_out.b0est = x(7);
    params_out.b1est = x(8);

    % update controller
    params_out.a0 = x(9);
    params_out.a1 = x(10);
    params_out.b0 = x(11);
    params_out.b1 = x(12);
    params_out.d0 = x(13);
    
end