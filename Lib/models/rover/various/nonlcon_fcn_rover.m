%% fcn
function [c, ceq] = nonlcon_fcn_rover(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;

    % init
    c = [];      

    % THETA TERMS
    % Theta = x(obs.init.params.pos_Gamma(1:3));
    % Theta_constr_down = -Theta;
    % c = [c; Theta_constr_down];
    
            
    % cons
    ceq = [];

end