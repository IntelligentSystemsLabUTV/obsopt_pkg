%% PARAMS_UPDATE_BATTERY
% file: params_update_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function updates the estimated parameters on a Van der 
% Pol oscillator
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_battery_tushar(params,x,t)
    % assign params
    params_out = params;          

    params_out.Voc = x(3,:);
    params_out.R0 = x(4,:);
    params_out.R1 = x(5,:);
    params_out.C1 = x(6,:);

    params_out.alpha_Voc = x(7,:);
    params_out.alpha_R0 = x(8,:);
    params_out.alpha_R1 = x(9,:);
    params_out.alpha_C1 = x(10,:);
    
    params_out.beta_Voc = x(11,:);
    params_out.beta_R0 = x(12,:);
    params_out.beta_R1 = x(13,:);
    params_out.beta_C1 = x(14,:);
    
    params_out.gamma_Voc = x(15,:);
    params_out.gamma_R0 = x(16,:);
    params_out.gamma_R1 = x(17,:);
    params_out.gamma_C1 = x(18,:);
    
    
end