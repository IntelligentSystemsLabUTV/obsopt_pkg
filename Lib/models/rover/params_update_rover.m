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

    if ~params.EKF

        params_out.theta(1:5) = x(params.pos_Gamma(3:7));
%         params_out.alpha(1:end) = x(params.pos_Gamma(10:end));  

        % normal beta
        params_out.beta(1) = x(params.pos_Gamma(8));
        params_out.beta(2) = x(params.pos_Gamma(9));

        % passband
        params_out.C(1:2) = x(params.pos_Gamma(1:2));
%         params_out.beta(1) = 1;
%         params_out.beta(2) = -params_out.C(2);
%         params_out.beta = params_out.theta(2)*params_out.beta;
    end
        
    
end