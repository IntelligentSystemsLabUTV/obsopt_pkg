%% fcn
function [c, ceq] = nonlcon_fcn(xopt,xnonopt,obs)

    params = obs.init.params;
    
    % create full state vector
    x = zeros(params.dim_state,1);
    x(params.opt_vars) = xopt;
    x(params.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;
    
    % constraints   
    c = [];
    ceq = [];

    % Gamma positive definite
    Gamma = x(params.GammaPos);
    GAMMA = zeros(params.nu, params.nu);
    tmp = 0;
    for i = 1 : params.nu
      for j = i : params.nu
        tmp = tmp + 1;
        GAMMA(i, j) = Gamma(tmp);
        GAMMA(j, i) = Gamma(tmp);
      end
    end
    c = [c, -(eig(GAMMA) - tol)];

    % Psi Hurwitz
    Psi = x(params.PsiPos);
    den_An = cellmat(params.m, params.nu, 1, params.eta+2);
    for k = 1 : params.eta+2
      for j = 1 : params.nu
        for i = 1 : params.m
          den_An{i, j}(k) = Psi(params.nu*(k-1) + j);
        end
      end
    end     
    for j = 1 : params.nu
      for i = 1 : params.m
        c = [c; real(roots(den_An{i, j})) + tol];
      end
    end

end