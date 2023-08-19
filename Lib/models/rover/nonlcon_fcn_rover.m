%% fcn
function [c, ceq] = nonlcon_fcn_rover(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 0*obs.init.myoptioptions.ConstraintTolerance;
    
    % eigs pos
    for i=1:size(obs.init.params.EIGPOS,1)
        EIGPOS(i) = mean(nonzeros(abs(obs.init.params.EIGPOS(i,:))));        
    end
    % eigs quat
    for i=1:size(obs.init.params.EIGQUAT,1)
        EIGQUAT(i) = mean(nonzeros(abs(obs.init.params.EIGQUAT(i,:))));        
    end
    EIGPOSmax = max(EIGPOS);
    EIGQUATmax = max(EIGQUAT);
    

    % init
    c = []; 
    ceq = [];

            
    % cons
    % temp = EIGPOSmax - (1 - tol);
    % c = [c;temp];
    % temp = EIGQUATmax - (1 - tol);
    % c = [c;temp];

end