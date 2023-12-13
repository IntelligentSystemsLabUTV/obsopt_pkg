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
function params_out = params_update_TCV_Zaccarian(params,x)

    % assign params
    params_out = params;  
    
    % update psi

    %%% update the denominator %%%
    params_out.Psi = x(end-params.NumGamma-params.NumPsi+1:end-params.NumGamma)';

    % update An
    params_out.W_An = tf(params.num_An, params_out.Psi);
    params_out.Anstar = dcgain(params_out.W_An);
    params_out.sys_An = ss(params_out.W_An);
    params_out.A_an = params_out.sys_An.A;
    params_out.B_an = params_out.sys_An.B;
    params_out.C_an = params_out.sys_An.C;
    params_out.D_an = params_out.sys_An.D;
    params_out.sys_An.InputName = 'v';
    params_out.sys_An.OutputName = 'ya';    
        
    % update gamma
    safeeps = 1e-10;
    params_out.gamma = x(end+1-params.NumGamma:end);
    params_out.Gamma = diag(params_out.gamma+safeeps);
    params_out.A_op = params_out.Gamma*params.sys_op_def.A;
    params_out.B_op = params_out.Gamma*params.sys_op_def.B;
    params_out.C_op = params.sys_op_def.C;
    params_out.D_op = params.sys_op_def.D;
    params_out.sys_op = ss(params_out.A_op,params_out.B_op,params_out.C_op,params_out.D_op);  
    params_out.sys_op.InputName = 'yc';
    params_out.sys_op.OutputName = 'v';
            
    % update CL sys
    i = params.traj;    
    params_out.sys_SumAll = sumblk('u = yc + ya',params.m);  
    params_out.sys_pert(i).sys_CL_Allu = connect(params.sys_Sum,params.sys_C_err,params_out.sys_op,params_out.sys_An,params.sys_SumAll,params.sys_pert(i).sys_P,'r','u');
    params_out.sys_pert(i).sys_CL_All = connect(params.sys_Sum,params.sys_C_err,params_out.sys_op,params_out.sys_An,params.sys_SumAll,params.sys_pert(i).sys_P,'r','y');  


end