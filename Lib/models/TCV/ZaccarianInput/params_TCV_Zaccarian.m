%% PARAMS_OSCILLATOR_VDP
% file: params_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function initialises the parameters for an unstable LTI
% model
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_TCV_Zaccarian(varargin)

    % case 1 = strong
    % case 2 = static annihilator
    % case 3 = dyn annihilator
    params.case = 3;
    
    % Zaccarian
    % generate system
    params.A_def = [-0.157 -0.094; ...
                -0.416 -0.45];
    params.B_def = [0.87 0.253 0.743; ...
                0.39 0.354 0.65];
    params.C_def = [0 1];
    params.D_def = [0 0 0];
    params.sys_P_def = ss(params.A_def,params.B_def,params.C_def,params.D_def);
    
    % generate dimensions
    params.n = size(params.A_def,1);
    params.m = size(params.B_def,2);
    params.p = size(params.C_def,1);
    
    % companion form
    params.A = [-0.607, 1;
                      -0.03155, 0];
    params.B = [0.39 0.354 0.65;
                      -0.3007 -0.04967 -0.207];
    params.C = [1 0];
    params.D = [0 0 0]; 
    params.sys_P = ss(params.A, params.B, params.C, params.D);       

    % generate subsystem bar
    params.q_pos = 1;    
    params.q = length(params.q_pos); 
    params.eta = [];
    
    % reference signal system (Sigma_r)
    params.dim_state_r = size(params.C,1);
    params.Ar = zeros(params.dim_state_r);
    params.Br = 0;
    params.Cr = zeros(params.dim_state_r);
    params.Dr = 1;   
    params.sys_R = ss(params.Ar,params.Br,params.Cr,params.Dr);    
    params.sys_R.InputName = 'ref';
    params.sys_R.OutputName = 'r';
    params.sys_Sum = sumblk('e = r - y',params.q);
                    
    % input stuff
    params.dim_input = params.m;    
    
    % controller system (Sigma_c)    
    params.dim_state_c = 4;
    params.Ac = [-1.57 0.5767 0.822 -0.65;...
                 -0.9 -0.501 -0.94 0.802; ...
                 0 1 -1.61 1.614; ...
                 0 0 0 0];
    params.Bcr = [0; 0; 0; 1];
    params.Bc = -params.Bcr;
    params.Cc = [1.81 -1.2 -0.46 0; ...
                 -0.62 1.47 0.89 0; ...
                 0 0 0 0];
    params.Dcr = [0; 0; 0];
    params.Dc = -params.Dcr;
    params.sys_C = ss(params.Ac,[params.Bc params.Bcr],params.Cc,[params.Dc params.Dcr]); 
    
    % only one input
    params.sys_C_err = ss(params.Ac,params.Bcr,params.Cc,params.Dcr);   
    params.sys_C_err.InputName = 'e';
    params.sys_C_err.OutputName = 'yc';
    
    % optimizer (Sigma_op) 
    if params.case == 1
        % input allocation Strong (Zaccarian)
        params.Bperp = null([params.B; params.D]);
        params.dim_state_op = size(params.Bperp,2);
        
        % Sigma op
        params.R = [1 1 1].*eye(params.dim_input);
        params.gamma = 1e-1;
        params.A_op = -params.gamma*params.Bperp'*params.R*params.Bperp;
        params.B_op = -params.gamma*params.Bperp'*params.R;        
        params.C_op = eye(params.dim_state_op);
        params.D_op = zeros(params.dim_state_op,params.m);
        params.sys_op = ss(params.A_op,params.B_op,params.C_op,params.D_op);
        
        % Sigma an
        params.dim_state_an = 1;
        params.A_an = zeros(params.dim_state_an);
        params.B_an = zeros(params.dim_state_an,params.dim_state_op);
        params.C_an = zeros(params.m,params.dim_state_an);
        params.D_an = params.Bperp;
        params.sys_An = ss(params.A_an,params.B_an,params.C_an,params.D_an);
        
        params.NumPsi = 0;
        params.Psi = [];

    elseif params.case == 2
        % input allocation weak static (Zaccarian)  
        params.Pstar = dcgain(params.sys_P);
        params.Pperp = null(params.Pstar);        
        params.dim_state_op = size(params.Pperp,2);
        
        % Sigma op
        params.R = [1 1 1].*eye(params.dim_input);
        params.gamma = 1e-1;
        params.A_op = -params.gamma*params.Pperp'*params.R*params.Pperp;
        params.B_op = -params.gamma*params.Pperp'*params.R;        
        params.C_op = eye(params.dim_state_op);
        params.D_op = zeros(params.dim_state_op,params.m);
        params.sys_op = ss(params.A_op,params.B_op,params.C_op,params.D_op);
        
        % Sigma an
        params.dim_state_an = 1;
        params.A_an = zeros(params.dim_state_an);
        params.B_an = zeros(params.dim_state_an,params.dim_state_op);
        params.C_an = zeros(params.m,params.dim_state_an);
        params.D_an = params.Pperp;
        params.sys_An = ss(params.A_an,params.B_an,params.C_an,params.D_an);
        
        params.NumPsi = 0;
        params.Psi = [];
        
    else
        % input allocation weak dynamic (Zaccarian)  
        % dynamic annihilator    
        [params.sys_An, params.num_An, params.den_An, params.N, params.N_An] = annihilator(params.sys_P); 
        params.A_an = params.sys_An.A;
        params.B_an = params.sys_An.B;
        params.C_an = params.sys_An.C;
        params.D_an = params.sys_An.D;
        params.dim_state_an = size(params.A_an,1);
        params.NumPsi = length(params.den_An);            
        params.Psi = params.den_An';
        % custom
        % params.Psi = [1.7748e-01 4.1587e+00 1.4709e+04 1.2118e+05]';        
                
        params.Anstar = dcgain(params.sys_An);        
        params.dim_state_op = size(params.Anstar,2);
        
        % Sigma op
        params.R = [1 1 1].*eye(params.dim_input);
        params.gamma = 1*ones(params.dim_state_op,1);
        params.NumGamma = params.dim_state_op;
        params.Gamma = diag(params.gamma);
        params.A_op = -params.Gamma*params.Anstar'*params.R*params.Anstar;
        params.B_op = -params.Gamma*params.Anstar'*params.R;        
        params.C_op = eye(params.dim_state_op);
        params.D_op = zeros(params.dim_state_op,params.m);
        params.sys_op = ss(params.A_op,params.B_op,params.C_op,params.D_op);                
        
    end    
    
    params.sys_An.InputName = 'v';
    params.sys_An.OutputName = 'ya';
    params.sys_op.InputName = 'yc';
    params.sys_op.OutputName = 'v';
    
                            
    % state dimension
    params.dim_state = params.dim_state_r + params.dim_state_c + params.dim_state_op + params.dim_state_an + params.n + params.NumPsi + params.NumGamma;        
    
    % initial condition
    % [x, xc, xop, xan, psi]
    params.X(1).val(:,1) = [zeros(params.dim_state_r,1); zeros(params.dim_state_c,1); zeros(params.dim_state_op,1); zeros(params.dim_state_an,1); 0*ones(params.n,1); params.Psi; params.gamma];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [params.dim_state_r + params.dim_state_c + params.dim_state_op + params.dim_state_an + params.n + 1:params.dim_state];
    
    % which vars am I optimising
    params.opt_vars = [params.dim_state_r + params.dim_state_c + params.dim_state_op + params.dim_state_an + params.n + 1:params.dim_state-params.NumGamma - 1];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % out vars
    params.OutDim = 1;
    params.OutDim_compare = [1];
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:(params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an);
    params.plot_params = (params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + 1):params.dim_state;   
    
    % number of reference trajectories (under development)
    params.Ntraj = 5;
    params.traj = 1;
    params.optimising = 0;
    
    % perturbed models
    params.sys_pert(1).A = params.A;
    params.sys_pert(1).B = params.B;
    params.sys_pert(1).C = params.C;
    params.sys_pert(1).D = params.D;
    params.sys_pert(1).sys_P = params.sys_P;
    
    % pert perc
    params.pert_perc = 0.05;
    for i=2:params.Ntraj
        params.sys_pert(i).A = [[params.A(1:end-1,1).*(1+params.pert_perc*randn(params.n-1,1)); params.A(end,1)] params.A(:,2:end)];
        params.sys_pert(i).B = [params.B(1:end-1,:).*(1+params.pert_perc*randn(params.n-1,params.m)); params.B(end, :)];
        params.sys_pert(i).C = params.C;
        params.sys_pert(i).D = params.D;
        params.sys_pert(i).sys_P = ss(params.sys_pert(i).A,params.sys_pert(i).B,params.sys_pert(i).C,params.sys_pert(i).D);
        
        params.X(i).val(:,1) = params.X(i-1).val(:,1);
    end
    
    %%% closed loop system %%%
    for i=1:params.Ntraj
        %%% no allocation closed loop %%%
        % plant and controller        
        params.sys_pert(i).sys_P.InputName = 'u';
        params.sys_pert(i).sys_P.OutputName = 'y';        
        params.sys_SumAll = sumblk('u = yc',params.m);        
        params.sys_pert(i).sys_CLu = connect(params.sys_Sum,params.sys_C_err,params.sys_SumAll,params.sys_pert(i).sys_P,'r','u');
        params.sys_pert(i).sys_CL = connect(params.sys_Sum,params.sys_C_err,params.sys_SumAll,params.sys_pert(i).sys_P,'r','y');         
        
        params.sys_SumAll = sumblk('u = yc + ya',params.m);  
        params.sys_pert(i).sys_CL_Allu = connect(params.sys_Sum,params.sys_C_err,params.sys_op,params.sys_An,params.sys_SumAll,params.sys_pert(i).sys_P,'r','u');
        params.sys_pert(i).sys_CL_All = connect(params.sys_Sum,params.sys_C_err,params.sys_op,params.sys_An,params.sys_SumAll,params.sys_pert(i).sys_P,'r','y');
        params.Perm = 1:20;
        params.sys_pert(i).sys_CL_Allu = xperm(params.sys_pert(i).sys_CL_Allu,params.Perm);
        params.sys_pert(i).sys_CL_All = xperm(params.sys_pert(i).sys_CL_All,params.Perm);
        
    end
end