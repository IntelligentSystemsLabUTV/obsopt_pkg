    %% PARAMS_OSCILLATOR_VDP
% file: params_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function initialises the parameters for an unstable LTI
% model
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_TCV_Zaccarian_outAll_discrete(varargin)

    params.discrete = varargin{1}{4};
    Ts = varargin{1}{5};
    
    % Zaccarian
    % generate system
    params.A = [-0.157 -0.094; ...
                -0.416 -0.45];
    params.B = [0.87 0.253 0.743; ...
                0.39 0.354 0.65];
    params.C = [0 1; 1 0];
    params.D = [0 0 0; 0 0 0];
    params.sys_P = ss(params.A,params.B,params.C,params.D); 
    if params.discrete == 1
        params.sys_P_CT = params.sys_P;
        params.A_CT = params.A;
        params.B_CT = params.B;
        params.C_CT = params.C;
        params.D_CT = params.D;
        
        params.sys_P = c2d(params.sys_P,Ts);        
        params.A = params.sys_P.A;
        params.B = params.sys_P.B;
        params.C = params.sys_P.C;
        params.D = params.sys_P.D;
    end

    % generate dimensions
    params.n = size(params.A,1);
    params.m = size(params.B,2);
    params.p = size(params.C,1);

    % generate subsystem bar
    params.q_pos = 1;    
    params.q = length(params.q_pos); 
    params.Cbar = params.C(params.q_pos,:);
    params.Dbar = params.D(params.q_pos,:);    
    params.sys_Pbar = ss(params.A,params.B,params.Cbar,params.Dbar);    
    if params.discrete == 1        
        params.Cbar_CT = params.C_CT(params.q_pos,:);
        params.Dbar_CT = params.D_CT(params.q_pos,:);
        params.sys_Pbar_CT = ss(params.A_CT,params.B_CT,params.Cbar_CT,params.Dbar_CT);
        params.sys_Pbar = c2d(params.sys_Pbar,Ts);      
    end
    
    % generate subsystem tilde        
    params.eta_pos = 1:params.p;
    params.eta_pos(params.q_pos) = [];
    params.eta = params.p-params.q;
    params.Ctilde = params.C(params.eta_pos,:);        
    params.Dtilde = params.D(params.eta_pos,:);
    params.sys_Ptilde = ss(params.A,params.B,params.Ctilde,params.Dtilde);
    if params.discrete == 1
        params.sys_Ptilde = c2d(params.sys_Ptilde,Ts);         
    end
    
    % reference signal system (Sigma_r)
    params.dim_state_r = size(params.C,1);
    params.Ar = zeros(params.dim_state_r);
    params.Br = 0;
    if params.discrete == 0
        params.Cr = eye(params.dim_state_r);
    else
        params.Cr = eye(params.dim_state_r);
    end
    params.Dr = 0;    
                    
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
    if params.discrete == 1
        params.sys_C = c2d(params.sys_C,Ts); 
        params.Ac = params.sys_C.A;
        params.Bc = params.sys_C.B(:,1:size(params.Bc,2));
        params.Bcr = params.sys_C.B(:,size(params.Bc,2)+1:end);
        params.Cc = params.sys_C.C;
        params.Dc = params.sys_C.D(:,1:size(params.Bc,2));
        params.Dcr = params.sys_C.D(:,size(params.Bc,2)+1:end);
    end
    
    % choose allocator
    params.staticAll = varargin{1}{3};
    
    % dynamic annihilator
    if params.discrete == 0
        [params.sys_An, params.W_An, params.N, params.N_An, params.num_An] = annihilator(params.sys_Pbar);            
    else
        [params.sys_An, params.W_An, params.N, params.N_An, params.num_An] = annihilator(params.sys_Pbar_CT);            
    end
    % get denominator and set numbers of parameters
    [~, D] = tfdata(params.W_An(1,1));
    params.NumPsi = length(D{1});    
    % params to be estimated
    for i=1:params.NumPsi
       tmpstr =  ['params.psi_', num2str(i) ' = D{1}(end-i+1);'];
       eval(tmpstr);       
    end
    params.Psi = reshape(D{1},params.NumPsi,1);
    % custom
    params.Psi = [2.7225e-01 1.4015e+01 1.0018e+03 4.1827e+04]';
    
    % output allocation         
    params.Pstar = dcgain(params.sys_P);
    params.Pstar_tilde = dcgain(params.sys_Ptilde);
    params.Pstar_bar = dcgain(params.sys_Pbar);
    
    if params.staticAll
        params.Pperp_bar = null(params.Pstar_bar);
    else
        params.Pperp_bar = dcgain(params.sys_An);
    end
    
    % optimizer (Sigma_op)    
    params.dim_state_op = size(params.Pperp_bar,2);          
    
    % annihilator (Sigma_an)
    if params.staticAll
        params.dim_state_an = 1;
        params.A_an = zeros(params.dim_state_an);
        params.B_an = zeros(params.dim_state_an,params.dim_state_op);
        params.C_an = zeros(params.dim_state_an);                          
        params.D_an = params.Pperp_bar;         
        params.sys_An = ss(params.A_an,params.B_an,params.C_an,params.D_an);
    else
        params.A_an = params.sys_An.A;
        params.B_an = params.sys_An.B;
        params.C_an = params.sys_An.C;
        params.D_an = params.sys_An.D;        
        params.dim_state_an = size(params.A_an,1);
    end   
    if params.discrete == 1
        params.sys_An = c2d(params.sys_An,Ts);
        params.A_an = params.sys_An.A;
        params.B_an = params.sys_An.B;
        params.C_an = params.sys_An.C;
        params.D_an = params.sys_An.D;  
    end
    params.Rcoef = 1e10;
    params.R = diag(params.Rcoef)*eye(params.eta); 
    params.gamma = -1e-2;
            
    % state dimension
    params.dim_state = params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + params.NumPsi; % + length(params.gamma) + length(params.Rcoef);
    
    % initial condition
    % [x, xc, xop, xan, psi]
    params.X(1).val(:,1) = [ones(params.dim_state_r,1); zeros(params.dim_state_c,1); zeros(params.dim_state_op,1); zeros(params.dim_state_an,1); 0*ones(params.n,1); params.Psi];
        %params.gamma; params.Rcoef; params.Psi];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + 1:params.dim_state];
    
    % which vars am I optimising
    params.opt_vars = [params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + 1:params.dim_state];    
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % out vars
    params.OutDim = 2;
    params.OutDim_compare = [params.q_pos];    
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:(params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an);
    params.plot_params = (params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + 1):params.dim_state;   
    
    % number of reference trajectories (under development)
    params.Ntraj = 20;
    
    % perturbed models
    params.sys_pert(1).A = params.A;
    params.sys_pert(1).B = params.B;
    params.sys_pert(1).C = params.C;
    params.sys_pert(1).D = params.D;
    params.sys_pert(1).sys = params.sys_P;
    
    % pert perc
    params.pert_perc = 0.05;
    for i=2:params.Ntraj
        params.sys_pert(i).A = params.A.*(1+params.pert_perc*randn(size(params.A)));
        params.sys_pert(i).B = params.B.*(1+params.pert_perc*randn(size(params.B)));
        params.sys_pert(i).C = params.C;
        params.sys_pert(i).D = params.D;
        params.sys_pert(i).sys = ss(params.sys_pert(i).A,params.sys_pert(i).B,params.sys_pert(i).C,params.sys_pert(i).D);        
        
        params.X(i).val(:,1) = params.X(1).val(:,1);
    end
end