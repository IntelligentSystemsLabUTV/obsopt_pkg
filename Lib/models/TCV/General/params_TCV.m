    %% PARAMS_OSCILLATOR_VDP
% file: params_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function initialises the parameters for an unstable LTI
% model
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_TCV(varargin)

        
    
    % Zaccarian
    params.Zaccarian = 1;
    if params.Zaccarian
        % generate system
        params.A = [-0.157 -0.094; ...
                    -0.416 -0.45];
        params.B = [0.87 0.253 0.743; ...
                    0.39 0.354 0.65];
        params.C = [0 1];
        params.D = [0 0 0];
        params.sys_P = ss(params.A,params.B,params.C,params.D);
        
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
        
        % generate subsystem tilde        
        params.eta_pos = 1:params.p;
        params.eta_pos(params.q_pos) = [];
        params.eta = params.p-params.q;
        params.Ctilde = params.C(params.eta_pos,:);        
        params.Dtilde = params.D(params.eta_pos,:);        
        params.sys_Ptilde = ss(params.A,params.B,params.Ctilde,params.Dtilde);
                          
    else
    
        %%% random sys %%
        % generate system
        params.sys_P = varargin{1}{2};
        params.A = params.sys_P.A;
        params.B = params.sys_P.B;    
        params.C = params.sys_P.C;
        params.D = params.sys_P.D;
        
        % generate dimensions
        params.n = size(params.A,1);
        params.m = size(params.B,2);
        params.p = size(params.C,1);
        
        % generate subsystem bar
        params.q_pos = varargin{1}{1};    
        params.q = length(params.q_pos);
        params.Cbar = params.C(params.q_pos,:);
        params.Dbar = params.D(params.q_pos,:);
        params.sys_Pbar = ss(params.A,params.B,params.Cbar,params.Dbar);
        
        % generate subsystem tilde        
        params.eta_pos = 1:params.p;
        params.eta_pos(params.q_pos) = [];
        params.eta = params.p-params.q;
        params.Ctilde = params.C(params.eta_pos,:);        
        params.Dtilde = params.D(params.eta_pos,:);        
        params.sys_Ptilde = ss(params.A,params.B,params.Ctilde,params.Dtilde);               
        
    end           
    
    % reference signal system (Sigma_r)
    params.dim_state_r = size(params.C,1);
    params.Ar = zeros(params.dim_state_r);
    params.Br = 0;
    params.Cr = eye(params.dim_state_r);
    params.Dr = 0;    
    
                
    % input stuff
    params.dim_input = params.m;    
    
    % controller system (Sigma_c)
    if params.Zaccarian
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
    else
        params.dim_state_c = 1;    
        params.Ac = zeros(params.dim_state_c);
        params.Bc = zeros(params.dim_state_c,params.q);
        params.Bcr = zeros(params.dim_state_c,params.q);
        params.Cc = zeros(params.dim_input,params.dim_state_c);
        params.Dc = eye(params.dim_input,params.q);
        params.Dcr = zeros(params.dim_input,params.q);
        params.sys_C = ss(params.Ac,[params.Bc params.Bcr],params.Cc,[params.Dc params.Dcr]);
    end
           
    %compute annihilator        
    params.sys_Pbar = ss(params.A,params.B,params.Cbar,params.Dbar);
    params.sys_Ptilde = ss(params.A,params.B,params.Ctilde,params.Dtilde);
        
    % choose allocator
    params.staticAll = varargin{1}{3};
                
    % dynamic allocator
    [params.sys_An, params.W_An, params.N, params.N_An] = annihilator(params.sys_Pbar);        
    
    % get denominator and set numbers of parameters
    [~, D] = tfdata(params.W_An(1,1));
    params.NumPsi = length(D{1});
    
    % params to be estimated
    for i=1:params.NumPsi
       tmpstr =  ['params.psi_', num2str(i) ' = D{1}(end-i+1);'];
       eval(tmpstr);       
    end
    params.Psi = reshape(D{1},params.NumPsi,1);  
    
    % static allocator
    params.Pstar = dcgain(params.sys_P);
    params.Pstar_tilde = dcgain(params.sys_Ptilde);
    params.Pstar_bar = dcgain(params.sys_Pbar);
    params.Pperp_bar = null(params.Pstar_bar);
    
    % input allocation (Zaccarian)
    params.Bperp = null([params.B; params.D]);
    
    % dynamic output annihilator
    params.Anstar = dcgain(params.sys_An);
    
    % optimizer (Sigma_op)
    if params.Zaccarian
        params.dim_state_op = size(params.Bperp,2); 
        params.W = eye(params.dim_input);
    else
        params.dim_state_op = size(params.Pperp_bar,2); 
    end            
    
    if params.staticAll
        % annihilator (Sigma_an)
        params.dim_state_an = 1;
        params.A_an = zeros(params.dim_state_an);
        params.B_an = zeros(params.dim_state_an,params.dim_state_op);
        params.C_an = zeros(params.dim_state_an);
        params.D_an = params.Pperp_bar;
        if params.Zaccarian
            params.Rzac = 1e1*eye(params.dim_state_op);   
            params.W = eye(params.dim_input);
            params.D_an_zac = params.Bperp;
        end
        params.R = 1e1*eye(params.eta);
        params.gamma = -1e-1;
    else
        params.A_an = params.sys_An.A;
        params.B_an = params.sys_An.B;
        params.C_an = params.sys_An.C;
        params.D_an = params.sys_An.D;
        params.dim_state_an = size(params.A_an,1);
        params.R = 1e2*eye(params.eta);
        params.gamma = -1e-1;
    end
            
    % state dimension
    params.dim_state = params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + params.NumPsi;        
    
    % initial condition
    % [x, xc, xop, xan, psi]
    params.X(1).val(:,1) = [ones(params.dim_state_r,1); zeros(params.dim_state_c,1); zeros(params.dim_state_op,1); zeros(params.dim_state_an,1); 0*ones(params.n,1); params.Psi];
    
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
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:(params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an);
    params.plot_params = (params.n + params.dim_state_c + params.dim_state_r + params.dim_state_op + params.dim_state_an + 1):params.dim_state;   
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
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
        
        params.X(i).val(:,1) = params.X(i-1).val(:,1);
    end
end