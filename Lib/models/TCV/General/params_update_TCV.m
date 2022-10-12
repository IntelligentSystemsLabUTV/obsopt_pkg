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
function params_out = params_update_TCV(params,x)

    % assign params
    params_out = params;    

    % update psi
    for i=1:params.NumPsi
        tmpstr =  ['params_out.psi_', num2str(i) ' = x(params.n+', num2str(i), ');'];
        eval(tmpstr); 
    end

    
end