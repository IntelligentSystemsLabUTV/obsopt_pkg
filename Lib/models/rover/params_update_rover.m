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
function params_out = params_update_rover(params,x)

    % assign params
    params_out = params; 

    params_out.theta = x(params.pos_Gamma(1:3));        
    params_out.gamma = x(params.pos_Gamma(4:19));  
        
    
end