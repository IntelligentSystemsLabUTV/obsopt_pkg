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
function params_out = params_update_control_test_ID(params,x)

    % assign params
    params_out = params;    

    % update model
    params_out.A1 = x(3);
    params_out.A2 = x(4);
    params_out.A3 = x(5);
    params_out.A4 = x(6);
    params_out.B1 = x(7);
    params_out.B2 = x(8);
    
end