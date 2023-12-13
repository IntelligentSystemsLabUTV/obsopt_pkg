%% PARAMS_UPDATE_OSCILLATOR_VDP
% file: params_update_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function updates the estimated parameters on a Van der 
% Pol oscillator
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_VolterraLotka(params,x)

    % assign params
    params_out = params;
    
    % update the nonlinearity
%     params_out.A_mu = x(4);
%     params_out.F_mu = x(5);
%     params_out.Phi_mu = x(6);    
end