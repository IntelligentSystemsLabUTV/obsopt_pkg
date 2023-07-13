%% PARAMS_UPDATE_ROVER
% file: params_update_rover.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function updates the estimated parameters on a rover
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_drone(params,x)

    % assign params
    params_out = params;   

    % update
    params_out.gamma = x(params_out.pos_gamma);
    
end