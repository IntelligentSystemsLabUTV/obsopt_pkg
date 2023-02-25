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
function params_out = params_update_TCV_Zaccarian_outAll(params,x)

    % assign params
    params_out = params; 
    
    if ~params.staticAll
        
%         params_out.Rcoef = x(end+1-params.NumPsi-length(params.Rcoef));
%         params_out.R = diag(params_out.Rcoef);
%         params_out.gamma = x(end+1-params.NumPsi-length(params.gamma)-length(params.Rcoef));

        % update psi
        for i=1:params.NumPsi
            tmpstr =  ['params_out.psi_', num2str(i) ' = x(end-', num2str(params.NumPsi-i), ');'];
            eval(tmpstr); 
        end

        %%% update the denominator %%%
        Psi = 'den = [';
        for i=1:params.NumPsi
            Psi = [Psi, 'params_out.psi_', num2str(i), ','];
        end
        Psi = [Psi(1:end-1), '];'];
        eval(Psi);    

        % update An
        W_An = tf(params.num_An, den);
        params_out.sys_An = ss(W_An);
        params_out.A_an = params_out.sys_An.A;
        params_out.B_an = params_out.sys_An.B;
        params_out.C_an = params_out.sys_An.C;
        params_out.D_an = params_out.sys_An.D;
    end
end