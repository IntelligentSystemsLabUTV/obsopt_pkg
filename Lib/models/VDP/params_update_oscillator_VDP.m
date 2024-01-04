%% params_update_oscillator_VDP
% file: params_update_oscillator_VDP.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function updates the estimated parameters on a Van der 
%              Pol oscillator
% INPUT:
%           Params: structure with all the necessary parameter to the model
%           x: state vector
% OUTPUT:
%           params_out: updated structure with the new model parameters
function params_out = params_update_oscillator_VDP(Params,x)

    % assign params
    params_out = Params;

    % update the Params.mu parameter
    Params.mu = x(3);
   
end