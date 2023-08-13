%% fcn
function [c, ceq] = nonlcon_fcn_rover(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;
    
    % eigs
    for i=1:3
        EIG(i) = max(abs(obs.init.params.EIG(i,:)));
    end
    EIG = max(EIG);
    

    % init
    c = []; 
    ceq = [];

            
    % cons
    temp = EIG - (1 - tol);
    c = [c;temp];

end