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
    
%     params_out.K = x(params.pos_Gamma(1));
%     params_out.C = x(params.pos_Gamma(2:3));
%     params_out.L = x(params.pos_Gamma(4));
%     params_out.G = x(params.pos_Gamma(5:6));
%     params_out.alpha = x(params.pos_Gamma(7:end));     

    params_out.K = x(params.pos_Gamma(1:2));
    params_out.C = x(params.pos_Gamma(3:6));
    params_out.L = x(params.pos_Gamma(7:8));
    params_out.G = x(params.pos_Gamma(9:12));
    params_out.alpha = x(params.pos_Gamma(13:end));     
    
end