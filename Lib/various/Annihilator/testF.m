%%
function [F,Fall_init,Fall_opt] = testF(params_init,params_opt)

    % Op init
    Anstar_init = dcgain(params_init.sys_An);
    A_op_init = -params_init.gamma*Anstar_init'*params_init.R*Anstar_init;
    B_op_init = -params_init.gamma*Anstar_init'*params_init.R;
    C_op_init = eye(size(A_op_init));
    D_op_init = zeros(size(C_op_init,1),size(B_op_init,2));
    Sigma_op_init = ss(A_op_init,B_op_init,C_op_init,D_op_init);
    
    % Sigma_A
    Sigma_A_init = Sigma_op_init*params_init.sys_An;
    
    % Op optimised
    Anstar_opt = dcgain(params_opt.sys_An);
    A_op_opt = -params_opt.gamma*Anstar_opt'*params_opt.R*Anstar_opt;
    B_op_opt = -params_opt.gamma*Anstar_opt'*params_opt.R;
    C_op_opt = eye(size(A_op_opt));
    D_op_opt = zeros(size(C_op_opt,1),size(B_op_opt,2));
    Sigma_op_opt = ss(A_op_opt,B_op_opt,C_op_opt,D_op_opt);
    
    % Sigma_A
    Sigma_A_opt = Sigma_op_opt*params_opt.sys_An;
    
    % F
    L = params_init.sys_C*params_init.sys_P;
    F = L/(1+L);
    
    % Fall init
    Lall = params_init.sys_C*(1+Sigma_A_init)*params_init.sys_P;
    Fall_init = Lall/(1+Lall);
    
    % Fall init
    Lall = params_opt.sys_C*(1+Sigma_A_opt)*params_opt.sys_P;
    Fall_opt = Lall/(1+Lall);
    
end
