%% fcn
function [c, ceq] = nonlcon_fcn_rover(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
%     tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;
    tol = 2e-2;
    
    % init
    c = [];  
    ceq = [];

            
    % cons
    temp = (x(obs.init.params.pos_gamma(1)) + x(obs.init.params.pos_gamma(2))) - 1 - tol;%sum(x(obs.init.params.pos_gamma)) - 1 - tol;
    c = [c;temp];
    temp = 1 - tol - (x(obs.init.params.pos_gamma(1)) + x(obs.init.params.pos_gamma(2)));
    c = [c;temp];
    temp = -x(obs.init.params.pos_gamma(1)) + tol;
    c = [c;temp];
    temp = -x(obs.init.params.pos_gamma(2)) + tol;
    c = [c;temp];
end