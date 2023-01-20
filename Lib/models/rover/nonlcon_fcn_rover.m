%% fcn
function [c, ceq] = nonlcon_fcn_rover(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;
    
    % negative poles    
    c = [];
    C = x(obs.init.params.pos_Gamma(3:6));
    G = x(obs.init.params.pos_Gamma(9:12));
    for dim=1:obs.init.params.space_dim
        range = fliplr([1+(dim-1)*obs.init.params.space_dim:dim*obs.init.params.space_dim]);
        tmp(:,1) = real(roots([1 C(range)'])) + tol;
        tmp(:,2) = real(roots([1 G(range)'])) + tol;
        c = [c ; tmp(:,1); tmp(:,2)];
    end
    
            
    % cons
    ceq = [];

end