%% fcn
function [c, ceq] = nonlcon_fcn(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;
    
    % negative poles    
    c = [];
%     for traj = 1:obs.init.params.Ntraj
%         sys_sz = size(obs.init.params.sys_pert(traj).sys_CL_All.A,1);
%         tmp = -1e2*ones(sys_sz,1);
%         minss = minreal(obs.init.params.sys_pert(traj).sys_CL_All,[],false);
%         eig_sys = (real(eig(minss)) + tol);
%         tmp(1:length(eig_sys)) = eig_sys;
%         c = [c; tmp];        
%     end
    
    % gamma negative
%     c = [c; -(x(obs.init.params.GammaPos) - tol)];
    
    % PSI roots < 0
%     tmp = (real(roots(x(obs.init.params.PsiPos))) + tol);
%     c = [c; tmp];
            
    % cons
    ceq = [];

end